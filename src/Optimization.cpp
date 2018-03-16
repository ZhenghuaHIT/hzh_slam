/**
* HZH_SLAM
* Optimization类
*/


#include "Optimization.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "Convert.h"


namespace HZH_SLAM
{

/**
 * @brief Pose Only Optimization
 *
 * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
 * 只优化Frame的Tcw，不优化MapPoints的坐标
 *
 * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
 * 2. Edge:
 *     - g2o::EdgeSE3ProjectXYZOnlyPose()，BaseUnaryEdge
 *         + Vertex：待优化当前帧的Tcw
 *         + measurement：MapPoint在当前帧中的二维位置(u,v)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *     - g2o::EdgeStereoSE3ProjectXYZOnlyPose()，BaseUnaryEdge
 *         + Vertex：待优化当前帧的Tcw
 *         + measurement：MapPoint在当前帧中的二维位置(ul,v,ur)
 *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
 *
 * @param   pFrame Frame
 * @return  inliers数量
 */
int Optimization::PoseOptimization(Frame *pFrame)
{
    // 该优化函数主要用于Tracking线程中：运动跟踪、参考帧跟踪、地图跟踪、重定位

    // 步骤1：构造g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    // 步骤2：添加顶点：待优化当前帧的Tcw
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Convert::CvMat2SE3Quat(pFrame->m_Tcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->mi_KPLeftN;

    // for Stereo
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaStereo = sqrt(7.815);

    // 步骤3：添加一元边：相机投影模型
    {

    for(int i=0; i<N; i++)
    {
        MapPoints* pMP = pFrame->mvp_MapPoints[i];
        if(pMP)
        {
            nInitialCorrespondences++;
            pFrame->mvb_Outlier[i] = false;

            //SET EDGE
            Eigen::Matrix<double,3,1> obs;
            const cv::KeyPoint &kp = pFrame->mv_KPLeft[i];
            const float &kp_ur = pFrame->mv_uRight[i];
            obs << kp.pt.x, kp.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mv_InvLevelSigma2[kp.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = pFrame->m_K.at<float>(0,0);
            e->fy = pFrame->m_K.at<float>(1,1);
            e->cx = pFrame->m_K.at<float>(0,2);
            e->cy = pFrame->m_K.at<float>(1,2);
            e->bf = pFrame->mf_bf;
            cv::Mat Xw = pMP->m_WorldPos;
            e->Xw[0] = Xw.at<float>(0);
            e->Xw[1] = Xw.at<float>(1);
            e->Xw[2] = Xw.at<float>(2);

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
        }
    }
    }

    if(nInitialCorrespondences<3)
        return 0;

    // 步骤4：开始优化，总共优化四次，每次优化后，将观测分为outlier和inlier，outlier不参与下次优化
    // 由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
    // const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const float chi2Stereo[4]={46.89,31.26,15.63,7.815};
    const int its[4]={10,10,10,10};// 四次迭代，每次迭代的次数

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Convert::CvMat2SE3Quat(pFrame->m_Tcw));
        optimizer.initializeOptimization(0);// 对level为0的边进行优化
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvb_Outlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvb_Outlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->mvb_Outlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Convert::SE3Quat2CvMat(SE3quat_recov);
    pFrame->PoseMatrixInversion(pose);

    return nInitialCorrespondences-nBad;
}


void Optimization::LocalPoseOptimization(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{

    // Local KeyFrames: First Breadth Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;
    int mRKF = pKF->mi_curfId;
    vector<int> mvLocalKFId;
    if(mRKF<10)
    {
        for( ; mRKF!=-1; mRKF-- )
        {
            mvLocalKFId.push_back(mRKF);
        }
    }
    else
    {
        for(int i=0; i<10; i++)
        {
            mvLocalKFId.push_back(mRKF-i);
        }
    }
    int numsRKF = mvLocalKFId.size();
    for(int in=0; in<numsRKF; in++)
    {
        int xid = mvLocalKFId[in];
        KeyFrame* pLKF = pMap->mvp_KeyFrames[xid];
        lLocalKeyFrames.push_back(pLKF);
    }
    cout<<"Local BA KeyFrame number is: "<<lLocalKeyFrames.size()<<endl;


    // 步骤3：遍历lLocalKeyFrames中关键帧，将它们观测的MapPoints加入到lLocalMapPoints
    list<MapPoints*> lLocalMapPoints;
    vector<MapPoints*> vpMPsa;
    for(list<KeyFrame*>::const_iterator itKF=lLocalKeyFrames.begin(), itEndKF=lLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoints*> vpMPs = pKF->GetLocalMapPoints();

        for(vector<MapPoints*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoints* pMP = *itMP;
            if(!pMP)
                continue;
            else
            {
                pMP->m_LocalMPMark2=0;
                vpMPsa.push_back(pMP);
            }
        }
    }
    // 步骤2：将局部关键帧的MapPoints添加到mvpLocalMapPoints
    for(vector<MapPoints*>::const_iterator itMP=vpMPsa.begin(), itEndMP=vpMPsa.end(); itMP!=itEndMP; itMP++)
    {
        MapPoints* pMP = *itMP;
        if(!pMP)
            continue;
        // 防止重复添加局部MapPoint
        if(pMP->m_LocalMPMark2==pMP->mi_curfId)
            continue;
        if(!pMP->isBad())
        {
            lLocalMapPoints.push_back(pMP);
            pMP->m_LocalMPMark2=pMP->mi_curfId;
        }
    }
    cout<<"Local BA MapPoints number is: "<<lLocalMapPoints.size()<<endl;


    // Setup optimizer
    // 步骤5：构造g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    // 步骤6：添加顶点：Pose of Local KeyFrame
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Convert::CvMat2SE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mi_curfId);
        vSE3->setFixed(pKFi->mi_curfId==0);//第一帧位置固定
        optimizer.addVertex(vSE3);
        if(pKFi->mi_curfId>maxKFid)
            maxKFid=pKFi->mi_curfId;
    }

    // Set MapPoint vertices
    // 步骤7：添加3D顶点
    const int nExpectedSize = lLocalKeyFrames.size()*lLocalMapPoints.size();

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoints*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoints*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {

        // 添加顶点：MapPoint
        MapPoints* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Convert::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mi_curfId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();
        // Set edges
        // 步骤8：对每一对关联的MapPoint和KeyFrame构建边
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
                KeyFrame* pKFi = mit->first;

                // 删除顶点不再LocalKeyFrames中的连接关系
                int bvertex = false;
                for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
                {
                    KeyFrame* pKFo = *lit;
                    if(pKFi->mi_curfId==pKFo->mi_curfId)
                        bvertex = true;
                }
                if(bvertex == false)
                    continue;

                const cv::KeyPoint &kpUn = pKFi->mv_KPLeft[mit->second];
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKFi->mv_uRight[mit->second];

                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));              
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mi_curfId)));

                e->setMeasurement(obs);
                const float &invSigma2 = pKFi->mv_InvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberStereo);

                e->fx = pKFi->m_K.at<float>(0,0);
                e->fy = pKFi->m_K.at<float>(1,1);
                e->cx = pKFi->m_K.at<float>(0,2);
                e->cy = pKFi->m_K.at<float>(1,2);
                e->bf = pKFi->mf_bf;

                optimizer.addEdge(e);
                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKFi);
                vpMapPointEdgeStereo.push_back(pMP);
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
        {
            cout<<"Interript Local BA to accept KeyFrame "<<*pbStopFlag<<endl;
            return;
        }

    // 步骤9：开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);

//    bool bDoMore= true;

//    if(pbStopFlag)
//        if(*pbStopFlag)
//            bDoMore = false;

//    if(bDoMore)
//    {

//    // Check inlier observations
//    // 步骤10：检测outlier，并设置下次不优化
//    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
//    {
//        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
//        MapPoints* pMP = vpMapPointEdgeStereo[i];

//        if(pMP->isBad())
//            continue;

//        if(e->chi2()>7.815 || !e->isDepthPositive())
//        {
//            e->setLevel(1);
//        }

//        e->setRobustKernel(0);
//    }

//    // Optimize again without the outliers
//    // 步骤11：排除误差较大的outlier后再次优化
//    optimizer.initializeOptimization(0);
//    optimizer.optimize(10);

//    }

//    vector<pair<KeyFrame*,MapPoints*> > vToErase;
//    vToErase.reserve(vpEdgesStereo.size());

//    // Check inlier observations
//    // 步骤12：在优化后重新计算误差，剔除连接误差比较大的关键帧和MapPoint
//    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
//    {
//        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
//        MapPoints* pMP = vpMapPointEdgeStereo[i];

//        if(e->chi2()>7.815 || !e->isDepthPositive())
//        {
//            KeyFrame* pKFi = vpEdgeKFStereo[i];
//            vToErase.push_back(make_pair(pKFi,pMP));
//        }
//    }

//    // 连接偏差比较大，在关键帧中剔除对该MapPoint的观测
//    // 连接偏差比较大，在MapPoint中剔除对该关键帧的观测
//    if(!vToErase.empty())
//    {
//        for(size_t i=0;i<vToErase.size();i++)
//        {
//            KeyFrame* pKFi = vToErase[i].first;
//            MapPoints* pMPi = vToErase[i].second;
//            pKFi->EraseMapPointMatch(pMPi);
//            pMPi->EraseObservation(pKFi);
//        }
//    }

    // Recover optimized data
    // 步骤13：优化后更新关键帧位姿以及MapPoints的位置、平均观测方向等属性

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mi_curfId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        cv::Mat pose = Convert::SE3Quat2CvMat(SE3quat);
        pKF->SetPose(pose);
        int numchang = pKF->m_posechangmark;
        numchang = numchang+1;
        pKF->m_posechangmark = numchang;
    }

    //Points
    for(list<MapPoints*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoints* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mi_curfId+maxKFid+1));
        pMP->SetWorldPos(Convert::toCvMat(vPoint->estimate()));
    }

}



} //namespace HZH_SLAM
