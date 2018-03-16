/**
* HZH_SLAM
* Tracking类
*/

#include "Tracking.h"

using namespace std;

namespace HZH_SLAM
{

Tracking::Tracking(const string &xstr_ConfigFile, Map *xpMap):
    m_Velocity(cv::Mat::eye(4,4,CV_32F)),me_state(SYSTEM_STANDBY),mp_Map(xpMap)
{
    //从配置文件中加载参数

    // 1.——加载 相机 parameters——
    cv::FileStorage fSettings(xstr_ConfigFile, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    // 相机内参
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(m_K);

    // 图像矫正系数
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(m_DistCoef);

    // 双目摄像头baseline * fx
    mf_bf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;
    mi_MinFrames = 0;
    mi_MaxFrames = fps;

    m_ThDepth = mf_bf*(float)fSettings["ThDepth"]/fx;
    cout << endl << "Depth Threshold (Close/Far Points): " << m_ThDepth << endl;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;

    // 1:RGB 0:BGR
    int nRGB = fSettings["Camera.RGB"];
    mb_RGB = nRGB;

    // 2.——加载 ORB parameters——

    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈值 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl<<endl;

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mp_ORBextractorLeft = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mp_ORBextractorRight = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

}

void Tracking::StereoTrack(const cv::Mat &mat_imLeft, const cv::Mat &mat_imRight, const double &d_stamp)
{
    mMat_GrayLeft = mat_imLeft;
    mMat_GrayRight = mat_imRight;

    //1.彩色图转灰度图
    if(mMat_GrayLeft.channels()==3)
    {
        if(mb_RGB)
        {
            cvtColor(mMat_GrayLeft, mMat_GrayLeft, CV_RGB2GRAY);
            cvtColor(mMat_GrayRight, mMat_GrayRight, CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mMat_GrayLeft, mMat_GrayLeft, CV_BGR2GRAY);
            cvtColor(mMat_GrayRight, mMat_GrayRight, CV_BGR2GRAY);
        }
    }
    else if(mMat_GrayLeft.channels()==4)
    {
        if(mb_RGB)
        {
            cvtColor(mMat_GrayLeft, mMat_GrayLeft, CV_RGBA2GRAY);
            cvtColor(mMat_GrayRight, mMat_GrayRight, CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mMat_GrayLeft, mMat_GrayLeft, CV_BGRA2GRAY);
            cvtColor(mMat_GrayRight, mMat_GrayRight, CV_BGRA2GRAY);
        }
    }

    //2.创建Frame类
    mF_CurrentFrame = Frame(mMat_GrayLeft,mMat_GrayRight,d_stamp,
                                mp_ORBextractorLeft,mp_ORBextractorRight,m_K,m_DistCoef,mf_bf);

    //3.进行track
    Tracking::TrackCloseFrame();

    cout << fixed <<
          "Track Sucess And Current Frame"<<"("<<mF_CurrentFrame.mi_curfId<<") "<<"Pose: "
         << endl <<" "<<mF_CurrentFrame.m_Tcw.inv()
         <<endl<<"—————————————————————————————————"<<endl<<endl;
    return;

}

void Tracking::TrackCloseFrame()
{
    if(me_state == SYSTEM_STANDBY)
    {
        me_state = SYSTEM_NOT_INITIALIZED;
    }

    if(me_state == SYSTEM_NOT_INITIALIZED)
    {
        //进入初始化
        SystemInitialization();
        if(me_state!=SYSTEM_TRACKING_ON)
            return;
        cout<<" —— System Initializaion Success ——"<<endl;
    }
    else
    {
        bool bok = false;

        //帧间Tracking
        if(me_state==SYSTEM_TRACKING_ON)
        {
            bok = TrackWithMotionModel();
            if(!bok)
                bok = TrackWithReferenceFrame();
        }

        // 将最新的关键帧作为参考关键帧
        mF_CurrentFrame.mF_ReferenceKF = mF_ReferenceKF;

        // 局部地图Tracking
        if(bok)
            bok = TrackLocalMap();

        if(bok)
            me_state = SYSTEM_TRACKING_ON;
        else
            me_state = SYSTEM_LOST;

        if(bok)
        {
            // 更新恒速运动模型
            if(!mF_LastFrame.m_Tcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mF_LastFrame.m_Rwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mF_LastFrame.m_Ow.copyTo(LastTwc.rowRange(0,3).col(3));
                m_Velocity = mF_CurrentFrame.m_Tcw*LastTwc; // Tcl
            }
            else
                m_Velocity = cv::Mat::eye(4,4,CV_32F);

            // 删除临时生成的Mappoints
            int mnTemporal = mF_CurrentFrame.mvp_MapPoints.size();
            for(int i=0; i<mnTemporal; i++)
            {
                MapPoints* pMP = mF_CurrentFrame.mvp_MapPoints[i];
                if(pMP)
                {
                    if(pMP->m_IfTemporal)
                    {
                        mF_CurrentFrame.mvb_Outlier[i] = false;
                        mF_CurrentFrame.mvp_MapPoints[i] = static_cast<MapPoints*>(NULL);
                        pMP->m_IfTemporal = false;
                    }
                }
            }

            {
                int mnc = mF_CurrentFrame.mvp_MapPoints.size();
                int numsc = 0;
                for(int i=0; i<mnc; i++)
                {
                    if(mF_CurrentFrame.mvp_MapPoints[i]!=NULL)
                        numsc++;
                }
                cout<<"After Delete Temporal Mappoints in Update: "<<numsc<<endl;
            }

            for(list<MapPoints*>::iterator lit = mlp_TemporalPoints.begin(), lend =  mlp_TemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoints* pMP = *lit;
                delete pMP;
            }
            mlp_TemporalPoints.clear();


            // 判断是否成为关键帧 是则创建关键帧
            if(NeedNewKeyFrame())
            {
                cout<<"Whether System Needs a New KeyFrame： True"<<endl;
                CreateNewKeyFrame();
                // 注释
                {
                    int mnd = mF_CurrentFrame.mvp_MapPoints.size();
                    int numsd = 0;
                    for(int i=0; i<mnd; i++)
                    {
                        if(mF_CurrentFrame.mvp_MapPoints[i]!=NULL)
                            numsd++;
                    }
                    cout<<"After Create New KeyFrame Mappoints: "<<numsd<<endl;
                }
            }
            else
                cout<<"Whether System Needs a New KeyFrame： False"<<endl;

            // 删除外点
            for(int i=0; i<mF_CurrentFrame.mi_KPLeftN;i++)
            {
                if(mF_CurrentFrame.mvp_MapPoints[i] && mF_CurrentFrame.mvb_Outlier[i])
                {
                     mF_CurrentFrame.mvp_MapPoints[i]=static_cast<MapPoints*>(NULL);
                     mF_CurrentFrame.mvb_Outlier[i] = false;
                }

            }

            {
                int mna = mF_CurrentFrame.mvp_MapPoints.size();
                int numsa = 0;
                for(int i=0; i<mna; i++)
                {
                    if(mF_CurrentFrame.mvp_MapPoints[i]!=NULL)
                        numsa++;
                }
                cout<<"After Delete Outliers Again Mappoints: "<<numsa<<endl;
            }

         }

        if(me_state==SYSTEM_LOST)
        {
            cout << "Track Lost, Reseting..." << endl;
            //mpSystem->Reset();
            exit(0);
        }

        if(!mF_CurrentFrame.mF_ReferenceKF)
            mF_CurrentFrame.mF_ReferenceKF = mF_ReferenceKF;

        mF_LastFrame = Frame(mF_CurrentFrame);
    }  
}

void Tracking::SetLocalMapper(LocalMap *pLocalMapper)
{
    mp_LocalMapper = pLocalMapper; //LocalMap线程与Track挂钩
}

void Tracking::SystemInitialization()
{

    if(mF_CurrentFrame.mi_KPLeftN>500)
    {
        // 步骤1：设定初始位姿
        mF_CurrentFrame.m_Tcw = cv::Mat::eye(4,4,CV_32F);
        mF_CurrentFrame.PoseMatrixInversion(mF_CurrentFrame.m_Tcw); //求逆

        // 步骤2：将当前帧构造为初始关键帧
        KeyFrame* pKF_ini = new KeyFrame(mF_CurrentFrame);

        // 步骤4：为每个特征点构造MapPoint 2.3
        for(int i=0; i<mF_CurrentFrame.mi_KPLeftN;i++)
        {
            float z = mF_CurrentFrame.mv_Depth[i];
            if(z>0)
            {
                // 步骤4.1：通过反投影得到该特征点的3D坐标
                cv::Mat KPw = mF_CurrentFrame.KPProjectWorld(i);
                // 步骤4.2：将3D点构造为MapPoint
                MapPoints* p_MP = new MapPoints(KPw,pKF_ini,i); // 3D点，初始KF

                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                p_MP->AddObservation(pKF_ini,i);
                // 步骤4.3：为该MapPoint添加属性：
                // b.从众多观测到该MapPoint的特征点中挑选区分读最高的描述子
                //p_MP->ComputeDistinctiveDescriptors();
                // c.更新该MapPoint平均观测方向以及观测距离的范围
                //p_MP->UpdateNormalAndDepth();

                // 步骤4.4：在地图中添加该MapPoint
                mp_Map->MapAddMapPoint(p_MP); // 类型set存放MP

                // 步骤4.5：表示该KeyFrame的哪个特征点可以观测到哪个3D点
                pKF_ini->KFAddMapPoint(p_MP,i); // 类型vector存放MP

                // 步骤4.6：将该MapPoint添加到当前帧的mvpMapPoints中
                // 为当前Frame的特征点与MapPoint之间建立索引
                mF_CurrentFrame.mvp_MapPoints[i]=p_MP; // 不能插入的已在frame初始化时置NULL

            }
        }

        // 步骤3：在Map&LocalMap中添加该初始关键帧
        mp_Map->MapAddKeyFrame(pKF_ini); // 类型set存放KF
        mp_LocalMapper->LocalMapAddKeyFrame(pKF_ini); // 类型list存放KF

        cout<<"Map 中的 MP size is : "<<mp_Map->msp_MapPoints.size()<<endl;
        cout << " —— New map created！—— " << endl;

        // 前后帧传递
        mF_LastFrame = Frame(mF_CurrentFrame);
        mi_LastKeyFrameId=mF_CurrentFrame.mi_curfId;
        mF_LastKeyFrame = pKF_ini;
        // 参考关键帧
        mF_ReferenceKF = pKF_ini;
        mF_CurrentFrame.mF_ReferenceKF = pKF_ini;

        me_state=SYSTEM_TRACKING_ON;
    }

}

bool Tracking::TrackWithMotionModel()
{
    int mnb = mF_LastFrame.mvp_MapPoints.size();
    int numsb = 0;
    for(int i=0; i<mnb; i++)
    {
        if(mF_LastFrame.mvp_MapPoints[i]!=NULL)
            numsb++;
    }

    // 步骤1：对于双目或rgbd摄像头，根据深度值为上一关键帧生成新的MapPoints
    AddLastFrameTMP();

    int mna = mF_LastFrame.mvp_MapPoints.size();
    int numsa = 0;
    for(int i=0; i<mna; i++)
    {
        if(mF_LastFrame.mvp_MapPoints[i]!=NULL)
            numsa++;
    }
    cout<<"Last Frame Mappoints Number: "<<numsb<<endl<<
          "After Update Last Frame Mappoints Number: "<<numsa<<endl;

    // MotionModel估计当前帧的位姿
    mF_CurrentFrame.PoseMatrixInversion(m_Velocity*mF_LastFrame.m_Tcw);

    // 特征配准 Mappoints传递
    Registration* mR_Registration = new Registration();
    int nmatches = mR_Registration->FramesProjection(mF_CurrentFrame,mF_LastFrame);

    // 步骤3：优化位姿
    Optimization::PoseOptimization(&mF_CurrentFrame);

    // 步骤4：优化位姿后剔除outlier的mvpMapPoints
    int nmatchesMap = 0;
    for(int i =0; i<mF_CurrentFrame.mi_KPLeftN; i++)
    {
        if(mF_CurrentFrame.mvp_MapPoints[i])
        {
            if(mF_CurrentFrame.mvb_Outlier[i])
            {
                //MapPoints* pMP = mF_CurrentFrame.mvp_MapPoints[i];

                mF_CurrentFrame.mvp_MapPoints[i]=static_cast<MapPoints*>(NULL);
                mF_CurrentFrame.mvb_Outlier[i]=false;
                //pMP->mbTrackInView = false;
                nmatches--;
            }
            else
                nmatchesMap++;
        }
    }

    //注释
    if(nmatchesMap>=50)
        cout<<"Track With Motion Model Success and Inliers of Mappoints is: "<<nmatchesMap<<endl;
    else
        cout<<"Track With Motion Model Fail and Inliers of Mappoints is: "<<nmatchesMap<<endl;

    return nmatchesMap>=50;
}

void Tracking::AddLastFrameTMP()
{
    // 如果上一帧为关键帧则退出
    if(mi_LastKeyFrameId==mF_LastFrame.mi_curfId)
        return;

    // 步骤2.1：得到上一帧有深度值的特征点
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mF_LastFrame.mi_KPLeftN);

    for(int i=0; i<mF_LastFrame.mi_KPLeftN;i++)
    {
        float z = mF_LastFrame.mv_Depth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    // 步骤2.2：按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // 步骤2.3：将距离比较近的点包装成MapPoints
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        MapPoints* pMP = mF_LastFrame.mvp_MapPoints[i];

        if(!pMP)
        {
            cv::Mat x3D = mF_LastFrame.KPProjectWorld(i);
            MapPoints* pNewMP = new MapPoints(x3D,&mF_LastFrame,i);

            mF_LastFrame.mvp_MapPoints[i]=pNewMP; // 添加新的MapPoint

            pNewMP->m_IfTemporal = true;

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            mlp_TemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>m_ThDepth && nPoints>100)
            break;
    }
}

bool Tracking::NeedNewKeyFrame()
{

    // 步骤4：查询局部地图管理器是否繁忙
    bool bLocalMappingIdle = mp_LocalMapper->AcceptKeyFrames();

    // 步骤5：对于双目或RGBD摄像头，统计总的可以添加的MapPoints数量和跟踪到地图中的MapPoints数量
    int nMappoints = 0;
    int nTotal= 0;
    for(int i =0; i<mF_CurrentFrame.mi_KPLeftN; i++)
    {
        if(mF_CurrentFrame.mv_Depth[i]>0 && mF_CurrentFrame.mv_Depth[i]<m_ThDepth)
        {
            nTotal++;// 总的可以添加mappoints数
            if(mF_CurrentFrame.mvp_MapPoints[i])
                nMappoints++;
        }
    }

    const float ratioMap = (float)nMappoints/(float)(std::max(1,nTotal));
    //const float ratioInlier = (float)mi_FrameInliers/(float)(std::max(1,nTotal));

    // MapPoints中和地图关联的比例阈值
    float thMapRatio = 0.35f;
    if(mi_FrameInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // 很长时间没有插入关键帧
    const bool c1a = mF_CurrentFrame.mi_curfId>=mi_LastKeyFrameId+mi_MaxFrames;

    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // localMapper处于空闲状态
     //const bool c1b = (mF_CurrentFrame.mi_curfId>=mi_LastKeyFrameId+mi_MinFrames && bLocalMappingIdle);
    const bool c1b = false;

    // Condition 1c: tracking is weak
    // 跟踪即将失败
    // const bool c1c = (ratioInlier<0.15 || ratioMap<0.3f);
    const bool c1c =  ratioMap<0.2f ;

    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // 阈值比c1c要高，与之前参考帧（最近的一个关键帧）重复度不是太高
    // const bool c2 = (ratioMap<thMapRatio && mi_FrameInliers>15);
    const bool c2 = ratioMap<thMapRatio ;

    //cout<<"bLocalMappingIdle "<<bLocalMappingIdle<<endl;

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            // 中断BA插入KF
            mp_LocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{

    // 步骤1：将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mF_CurrentFrame);

    mF_ReferenceKF = pKF;
    mF_CurrentFrame.mF_ReferenceKF = pKF;

    // 步骤3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
    // 根据Tcw计算mRcw、mtcw和mRwc、mOw
    mF_CurrentFrame.PoseMatrixInversion(mF_CurrentFrame.m_Tcw);

    // 步骤3.1：得到当前帧深度小于阈值的特征点
    // 创建新的MapPoint, depth < mThDepth
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mF_CurrentFrame.mi_KPLeftN);
    for(int i=0; i<mF_CurrentFrame.mi_KPLeftN; i++)
    {
        float z = mF_CurrentFrame.mv_Depth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(!vDepthIdx.empty())
    {
        // 步骤3.2：按照深度从小到大排序
        sort(vDepthIdx.begin(),vDepthIdx.end());

        // 步骤3.3：将距离比较近的点包装成MapPoints
        int nPoints = 0;
        for(size_t j=0; j<vDepthIdx.size();j++)
        {
            int i = vDepthIdx[j].second;

            MapPoints* pMP = mF_CurrentFrame.mvp_MapPoints[i];

            if(!pMP)
            {
                cv::Mat KPw = mF_CurrentFrame.KPProjectWorld(i);
                MapPoints* pNewMP = new MapPoints(KPw,pKF,i);

                // 这些添加属性的操作是每次创建MapPoint后都要做的
                pNewMP->AddObservation(pKF,i);
                pKF->KFAddMapPoint(pNewMP,i);
                //pNewMP->ComputeDistinctiveDescriptors();
                //pNewMP->UpdateNormalAndDepth();
                mp_Map->MapAddMapPoint(pNewMP);

                mF_CurrentFrame.mvp_MapPoints[i]=pNewMP;
                nPoints++;
            }
            else
            {
                nPoints++;
            }
            if(vDepthIdx[j].first>m_ThDepth && nPoints>100)
                break;
        }
    }

    mp_LocalMapper->LocalMapAddKeyFrame(pKF);
    mi_LastKeyFrameId = mF_CurrentFrame.mi_curfId;
    mF_LastKeyFrame = pKF;
    //mp_Map->MapAddKeyFrame(pKF); // LocalMapping 中添加

}

bool Tracking::TrackLocalMap()
{

    // 步骤1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
    UpdateLocalKeyFrames();
    UpdateLocalPoints();

    // 步骤2：在局部地图中查找与当前帧匹配的MapPoints
    ProjectLocalPoints();

    // Optimize Pose
    // 在这个函数之前，在Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel中都有位姿优化，
    // 步骤3：更新局部所有MapPoints后对位姿再次优化
    Optimization::PoseOptimization(&mF_CurrentFrame);
    mi_FrameInliers = 0;

    // 步骤3：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
    for(int i=0; i<mF_CurrentFrame.mi_KPLeftN; i++)
    {
        if(mF_CurrentFrame.mvp_MapPoints[i])
        {
            // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1
            if(!mF_CurrentFrame.mvb_Outlier[i])
                mi_FrameInliers++;
            else
            {
                mF_CurrentFrame.mvp_MapPoints[i]=static_cast<MapPoints*>(NULL);
                mF_CurrentFrame.mvb_Outlier[i]=false;
            }
        }
    }

    // 步骤4：决定是否跟踪成功
    if(mi_FrameInliers<1)
    {
        cout<<"Track With LocalMap Fail and Inliers: "<<mi_FrameInliers<<endl;
        return false;
    }
    else
    {
        cout<<"Track With LocalMap Success and Inliers: "<<mi_FrameInliers<<endl;
        return true;
    }

}

void Tracking::UpdateLocalKeyFrames()
{

    // 先清空局部关键帧
    mvp_LocalKeyFrames.clear();
    mvp_LocalKeyFrames.reserve(8);

    // 获取参考关键帧ID
    int mRKF = mF_CurrentFrame.mF_ReferenceKF->mi_curfId;

    // 局部关键帧ID，取7个关键帧
    vector<int> mvLocalKFId;
    if(mRKF<7)
    {
        for( ; mRKF!=-1; mRKF-- )
            mvLocalKFId.push_back(mRKF);

    }
    else
    {
        for(int i=0; i<8; i++)
            mvLocalKFId.push_back(mRKF-i);
    }
    int numsRKF = mvLocalKFId.size();
    for(int in=0; in<numsRKF; in++)
    {
        int xid = mvLocalKFId[in];
        KeyFrame* pLKF = mp_Map->mvp_KeyFrames[xid];
        mvp_LocalKeyFrames.push_back(pLKF);
        //cout<<"是否使用了优化过的pose： "<<pLKF->m_posechangmark<<endl;
    }
}

void Tracking::UpdateLocalPoints()
{
    // 步骤1：清空局部MapPoints
    mvp_LocalMapPoints.clear();

    // 步骤2：遍历局部关键帧mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_iterator itKF=mvp_LocalKeyFrames.begin(), itEndKF=mvp_LocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoints*> vpMPs = pKF->GetLocalMapPoints();

        for(vector<MapPoints*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoints* pMP = *itMP;
            if(!pMP)
                continue;
            else
                pMP->m_LocalMPMark=0;

        }

        // 步骤2：将局部关键帧的MapPoints添加到mvpLocalMapPoints
        for(vector<MapPoints*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoints* pMP = *itMP;
            if(!pMP)
                continue;
            // 防止重复添加局部MapPoint
            if(pMP->m_LocalMPMark==mF_CurrentFrame.mi_curfId)
                continue;
            if(!pMP->isBad())
            {
                mvp_LocalMapPoints.push_back(pMP);
                pMP->m_LocalMPMark=mF_CurrentFrame.mi_curfId;
            }
        }


    }

    cout<<"Track LocalMap Mappoints number: "<<mvp_LocalMapPoints.size()<<endl;
}

void Tracking::ProjectLocalPoints()
{
    // 特征点尺度预测
    for(vector<MapPoints*>::iterator vit=mvp_LocalMapPoints.begin(), vend=mvp_LocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoints* pMP = *vit;

        cv::Mat P = pMP->GetWorldPos();

        // 3D点P在相机坐标系下的坐标
        const cv::Mat Pc = mF_CurrentFrame.m_Rcw*P+mF_CurrentFrame.m_tcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        if(PcZ<0.0f)
            continue;

        // 将MapPoint投影到当前帧, 并判断是否在图像内
        const float fx = mF_CurrentFrame.m_K.at<float>(0,0);
        const float fy = mF_CurrentFrame.m_K.at<float>(1,1);
        const float cx = mF_CurrentFrame.m_K.at<float>(0,2);
        const float cy = mF_CurrentFrame.m_K.at<float>(1,2);
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        // 计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        // 世界坐标系下，相机到3D点P的向量, 向量方向由相机指向3D点P
        const cv::Mat PO = P-mF_CurrentFrame.m_Ow;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回
        cv::Mat Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<0.9)
            continue;

        // 根据深度预测尺度（对应特征点在一层）
        const int nPredictedLevel = pMP->PredictScale(dist,&mF_CurrentFrame);
        pMP->mi_TrackScaleLevel = nPredictedLevel;

        // Data used by the tracking
        // 标记该点将来要被投影
        pMP->mb_TrackInView = true;
        pMP->mf_TrackProjX = u;
        pMP->mf_TrackProjXR = u - mF_CurrentFrame.mf_bf*invz; //该3D点投影到双目右侧相机上的横坐标
        pMP->mf_TrackProjY = v;
        pMP->mf_TrackViewCos = viewCos;

    }

    // 特征配准 Mappoints传递
    Registration* mR_Registration = new Registration();
    mR_Registration->FramesProjection(mF_CurrentFrame,mvp_LocalMapPoints);

}

bool Tracking::TrackWithReferenceFrame()
{
    // MotionModel估计当前帧的位姿
    mF_CurrentFrame.PoseMatrixInversion(mF_LastFrame.m_Tcw);

    // 特征配准 Mappoints传递
    Registration* mR_Registration = new Registration();
    int nmatches = mR_Registration->FramesProjection(mF_CurrentFrame,mF_LastFrame);

    // 步骤3：优化位姿
    Optimization::PoseOptimization(&mF_CurrentFrame);

    // 步骤4：优化位姿后剔除outlier的mvpMapPoints
    int nmatchesMap = 0;
    for(int i =0; i<mF_CurrentFrame.mi_KPLeftN; i++)
    {
        if(mF_CurrentFrame.mvp_MapPoints[i])
        {
            if(mF_CurrentFrame.mvb_Outlier[i])
            {
                //MapPoints* pMP = mF_CurrentFrame.mvp_MapPoints[i];

                mF_CurrentFrame.mvp_MapPoints[i]=static_cast<MapPoints*>(NULL);
                mF_CurrentFrame.mvb_Outlier[i]=false;
                //pMP->mbTrackInView = false;
                nmatches--;
            }
            else
                nmatchesMap++;
        }
    }

    //注释
    if(nmatchesMap>=10)
        cout<<"Track With Reference Frame Success and Inliers of Mappoints is: "<<nmatchesMap<<endl;
    else
        cout<<"Track With Reference Frame Fail and Inliers of Mappoints is: "<<nmatchesMap<<endl;

    return nmatchesMap>=10;
}

} // namespace HZH_SLAM
