/**
* HZH_SLAM
* Frame类
*/

#include"Frame.h"
//#include"Converter.h"

using namespace std;

namespace HZH_SLAM
{

long unsigned int Frame::mi_lasfId = 0;

Frame::Frame()
{}

Frame::Frame(const Frame &frame):
mp_ORBextractorLeft(frame.mp_ORBextractorLeft),mp_ORBextractorRight(frame.mp_ORBextractorRight),m_K(frame.m_K.clone()),
m_DistCoef(frame.m_DistCoef.clone()),mf_bf(frame.mf_bf),mi_curfId(frame.mi_curfId),
mi_ScaleLevel(frame.mi_ScaleLevel),mf_ScaleFactor(frame.mf_ScaleFactor),mv_ScaleFactors(frame.mv_ScaleFactors),
mv_InvScaleFactors(frame.mv_InvScaleFactors),mv_LevelSigma2(frame.mv_LevelSigma2),mv_InvLevelSigma2(frame.mv_InvLevelSigma2),
mv_KPLeft(frame.mv_KPLeft),mv_KPRight(frame.mv_KPRight),m_DescriptorsLeft(frame.m_DescriptorsLeft.clone()),
m_DescriptorsRight(frame.m_DescriptorsRight.clone()),mi_KPLeftN(frame.mi_KPLeftN),mv_uRight(frame.mv_uRight),
mv_Depth(frame.mv_Depth),mvp_MapPoints(frame.mvp_MapPoints),mvb_Outlier(frame.mvb_Outlier),mF_ReferenceKF(frame.mF_ReferenceKF)
{
    if(!frame.m_Tcw.empty())
        PoseMatrixInversion(frame.m_Tcw);
}

Frame::Frame(const cv::Mat &xGrayLeft, const cv::Mat &xGrayRight, const double &xstamp, ORB_SLAM2::ORBextractor* xextractorLeft, ORB_SLAM2::ORBextractor* xextractorRight,
             cv::Mat &xK, cv::Mat &xDistCoef, const float &xbf)
    :mp_ORBextractorLeft(xextractorLeft), mp_ORBextractorRight(xextractorRight), m_K(xK.clone()), m_DistCoef(xDistCoef.clone()), mf_bf(xbf),mF_ReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mi_curfId = mi_lasfId++;

    // Scale Level Info
    mi_ScaleLevel = mp_ORBextractorLeft->GetLevels();
    mf_ScaleFactor = mp_ORBextractorLeft->GetScaleFactor();
    mv_ScaleFactors = mp_ORBextractorLeft->GetScaleFactors();
    mv_InvScaleFactors = mp_ORBextractorLeft->GetInverseScaleFactors();
    mv_LevelSigma2 = mp_ORBextractorLeft->GetScaleSigmaSquares();
    mv_InvLevelSigma2 = mp_ORBextractorLeft->GetInverseScaleSigmaSquares();

    // 对左右图像提取特征点
    thread threadLeft(&Frame::ExtractORB,this,0,xGrayLeft);
    thread threadRight(&Frame::ExtractORB,this,1,xGrayRight);
    threadLeft.join();
    threadRight.join();

    // 左图特征点数量
    mi_KPLeftN = mv_KPLeft.size();

//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // 特征点重建
    // ORB_ComputeStereoMatches(); HZH_ComputeStereoMatches(xGrayLeft, xGrayRight);
    ORB_ComputeStereoMatches();

//    int n=0,m=mv_Depth.size();
//    for (int i=0; i<m; i++) {if(mv_Depth[i]!=-1) n++;}
//    cout<<"The final matches number is "<<n<<endl;
//    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//    double time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//    cout<<"Match cost time is: "<<time<<endl;

    // 创建对应的mappoints
    mvp_MapPoints = vector<MapPoints*>(mi_KPLeftN,static_cast<MapPoints*>(NULL));
    mvb_Outlier = vector<bool>(mi_KPLeftN,false);

}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mp_ORBextractorLeft)(im,cv::Mat(),mv_KPLeft,m_DescriptorsLeft);
    else
        (*mp_ORBextractorRight)(im,cv::Mat(),mv_KPRight,m_DescriptorsRight);
}

/**
 * @brief 双目匹配
 *
 * ORB_SLAM2原始算法，耗时0.02s左右 提出特征点830左右 \n
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */
void Frame::ORB_ComputeStereoMatches()
{
    mv_uRight = vector<float>(mi_KPLeftN,-1.0f);
    mv_Depth = vector<float>(mi_KPLeftN,-1.0f);

    const int thOrbDist = (100+50)/2;

    const int nRows = mp_ORBextractorLeft->mvImagePyramid[0].rows; // 左图第一层金字塔行数

    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mv_KPRight.size(); // 右图特征点的数量


    for(int iR=0; iR<Nr; iR++)
    {
        // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
        const cv::KeyPoint &kp = mv_KPRight[iR];
        const float &kpY = kp.pt.y; // 右图特征的y值
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大
        const float r = 2.0f*mv_ScaleFactors[mv_KPRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR); // 金字塔第一层每一行将要搜索的行数
    }


    const float minD = 0;
    const float maxD = mf_bf;

    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(mi_KPLeftN);

    for(int iL=0; iL<mi_KPLeftN; iL++)
    {
        const cv::KeyPoint &kpL = mv_KPLeft[iL]; // 左图特征点
        const int &levelL = kpL.octave; // 左图特征点层数
        const float &vL = kpL.pt.y; // 左图特征点y值
        const float &uL = kpL.pt.x; // 左图特征的x值

        // 可能的匹配点 所在的行数
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD; // 最小匹配范围
        const float maxU = uL-minD; // 最大匹配范围

        if(maxU<0)
            continue;

        int bestDist = 100; // ?
        size_t bestIdxR = 0;

        // 每个特征点描述子占一行，建立一个指针指向iL特征点对应的描述子
        const cv::Mat &dL = m_DescriptorsLeft.row(iL);

        // Compare descriptor to right keypoints
        // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC]; // 取出可能对应的右图特征点序号
            const cv::KeyPoint &kpR = mv_KPRight[iR];

            // 仅对近邻尺度的特征点进行匹配
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = m_DescriptorsRight.row(iR);
                const int dist = ORB_DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
        // 最好的匹配的匹配误差存在bestDist，匹配点位置存在bestIdxR中

        // Subpixel match by correlation
        // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
            const float uR0 = mv_KPRight[bestIdxR].pt.x;
            const float scaleFactor = mv_InvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5; // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
            cv::Mat IL = mp_ORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1); // 11

            // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0-L-w; //这个地方是否应该是scaleduR0-L-w (wubo???)
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mp_ORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mp_ORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

                float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
                if(dist<bestDist)
                {
                    bestDist =  dist;// SAD匹配目前最小匹配偏差
                    bestincR = incR; // SAD匹配目前最佳的修正量（像素级）
                }

                vDists[L+incR] = dist; // 正常情况下，这里面的数据应该以抛物线形式变化 -5~5 ： 0-11
            }

            if(bestincR==-L || bestincR==L) // 整个滑动窗口过程中，SAD最小值不是以抛物线形式出现，SAD匹配失败，同时放弃求该特征点的深度
                continue;

            // Sub-pixel match (Parabola fitting)
            // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mv_ScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR); // bestincR+delta亚像素级，退回到一层金字塔

            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR); // 左图特征点x值 - 右图特征的x值

            if(disparity>=minD && disparity<maxD) // 最后判断视差是否在范围内
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                // depth 是在这里计算的
                // depth=baseline*fx/disparity
                mv_Depth[iL]=mf_bf/disparity;   // 深度
                mv_uRight[iL] = bestuR;       // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL)); // 该特征点SAD匹配最小匹配偏差
            }
        }
    }

    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median; // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mv_uRight[vDistIdx[i].second]=-1;
            mv_Depth[vDistIdx[i].second]=-1;
        }
    }
}

int Frame::ORB_DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

/**
 * @brief 双目匹配
 *
 * 删除了原有的匹配，改用Opencv自带的暴力匹配 耗时0.11s左右 特征点数720左右 \n
 * 采用ORB_SLAM2的SAD亚像素角点对右图进行优化 \n
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */
void Frame::HZH_ComputeStereoMatches(const cv::Mat &xGrayLeft, const cv::Mat &xGrayRight)
{

    mv_uRight = vector<float>(mi_KPLeftN,-1.0f);
    mv_Depth = vector<float>(mi_KPLeftN,-1.0f);

    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(mi_KPLeftN);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // 暴力匹配 0.1s
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(m_DescriptorsLeft, m_DescriptorsRight, matches);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"BruteForceMatcher cost time is: "<<time<<"  And Matches is:"<<matches.size()<<endl;

    // 可视化
    cv::Mat matchMat;
    cv::drawMatches(xGrayLeft, mv_KPLeft,xGrayRight, mv_KPRight, matches, matchMat);
    cv::imshow("Matches", matchMat);
    cv::waitKey(0);
    vector<cv::DMatch> goodmatches;


    int nmatches = matches.size();


    for (int i=0; i<nmatches; i++)
    {
        int iR = matches[i].trainIdx;
        int iL = matches[i].queryIdx;
        const cv::KeyPoint &kpR = mv_KPRight[iR];
        const cv::KeyPoint &kpL = mv_KPLeft[iL];

        const int &levelL = kpL.octave; // 左图特征点层数
        if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
            continue;

        const float &kpLY = kpL.pt.y; // 左图特征的y值
        const float &kpRY = kpR.pt.y;

        const float r = 2.0f*mv_ScaleFactors[kpR.octave];
        const int maxr = ceil(kpRY+r);
        const int minr = floor(kpRY-r);

        if(kpLY > maxr || kpLY < minr)
            continue;


        // coordinates in image pyramid at keypoint scale
        // kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
        const float uR0 = mv_KPRight[iR].pt.x;
        const float scaleFactor = mv_InvScaleFactors[kpL.octave];
        const float scaleduL = round(kpL.pt.x*scaleFactor);
        const float scaledvL = round(kpL.pt.y*scaleFactor);
        const float scaleduR0 = round(uR0*scaleFactor);

        // sliding window search
        const int w = 5; // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
        cv::Mat IL = mp_ORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
        IL.convertTo(IL,CV_32F);
        IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

        int bestDist = INT_MAX;
        int bestincR = 0;
        const int L = 5;
        vector<float> vDists;
        vDists.resize(2*L+1); // 11

        // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
        const float iniu = scaleduR0-L-w;
        const float endu = scaleduR0+L+w+1;
        if(iniu<0 || endu >= mp_ORBextractorRight->mvImagePyramid[kpL.octave].cols)
            continue;

        for(int incR=-L; incR<=+L; incR++)
        {
            // 横向滑动窗口
            cv::Mat IR = mp_ORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
            IR.convertTo(IR,CV_32F);
            IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

            float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
            if(dist<bestDist)
            {
                bestDist =  dist;// SAD匹配目前最小匹配偏差
                bestincR = incR; // SAD匹配目前最佳的修正量（像素级）
            }

            vDists[L+incR] = dist; // 正常情况下，这里面的数据应该以抛物线形式变化 -5~5 ： 0-11
        }

        if(bestincR==-L || bestincR==L) // 整个滑动窗口过程中，SAD最小值不是以抛物线形式出现，SAD匹配失败，同时放弃求该特征点的深度
            continue;

        // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
        // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
        // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
        const float dist1 = vDists[L+bestincR-1];
        const float dist2 = vDists[L+bestincR];
        const float dist3 = vDists[L+bestincR+1];

        const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

        // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
        if(deltaR<-1 || deltaR>1)
            continue;

        // 通过描述子匹配得到匹配点位置为scaleduR0
        // 通过SAD匹配找到修正量bestincR
        // 通过抛物线拟合找到亚像素修正量deltaR
        float bestuR = mv_ScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR); // bestincR+delta亚像素级，退回到一层金字塔

        // 这里是disparity，根据它算出depth
        float disparity = (kpL.pt.x-bestuR); // 左图特征点x值 - 右图特征的x值

        // depth=baseline*fx/disparity
        mv_Depth[i] = mf_bf/disparity;   // 深度
        mv_uRight[i] = bestuR;       // 匹配对在右图的横坐标
        vDistIdx.push_back(pair<int,int>(bestDist,i)); // 该特征点SAD匹配最小匹配偏差

        goodmatches.push_back(matches[i]);
    }

    // 可视化2
    cout<<"Good Matches is: "<<goodmatches.size()<<endl;
    cv::Mat matchMat2;
    cv::drawMatches(xGrayLeft, mv_KPLeft,xGrayRight, mv_KPRight, goodmatches, matchMat2);
    cv::imshow("Matches2", matchMat2);
    cv::waitKey(0);

    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median; // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mv_uRight[vDistIdx[i].second]=-1;
            mv_Depth[vDistIdx[i].second]=-1;
        }
    }

}

cv::Mat Frame::KPProjectWorld(const int &i)
{
    const float z = mv_Depth[i];
    if(z>0)
    {
        const float u = mv_KPLeft[i].pt.x;
        const float v = mv_KPLeft[i].pt.y;
        const float cx = m_K.at<float>(0,2);
        const float cy = m_K.at<float>(1,2);
        const float invfx = 1.0f/m_K.at<float>(0,0);
        const float invfy = 1.0f/m_K.at<float>(1,1);
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy; //像素坐标转相机坐标 内参矩阵
        cv::Mat KPcu = (cv::Mat_<float>(3,1) << x, y, z); //
        return m_Rwc*KPcu+m_Ow; // 相机坐标转世界坐标 外参矩阵
    }
    else
    {
        return cv::Mat();
    }
}

/**
 * @brief Computes rotation, translation and camera center matrices from the camera pose.
 *
 * 根据Tcw计算mRcw、mtcw和mRwc、mOw
 */
void Frame::PoseMatrixInversion(const cv::Mat &xTcw)
{
    m_Tcw = xTcw.clone();
    m_Rcw = m_Tcw.rowRange(0,3).colRange(0,3);
    m_Rwc = m_Rcw.t(); // RT
    m_tcw = m_Tcw.rowRange(0,3).col(3); // t
    m_Ow = -m_Rcw.t()*m_tcw; // T-1 中的-RTt
}

vector<size_t> Frame::GetKPInCurF(const float &xu, const float  &xv, const float  &xr, const int &xminLevel, const int &xmaxLevel)
{
    vector<size_t> vIndices;
    vIndices.reserve(mi_KPLeftN);

    const int nMinX = max(0,(int)floor(xu-xr));
    const int nMaxX = min(640,(int)ceil(xu+xr));
    const int nMinY = max(0,(int)floor(xv-xr));
    const int nMaxY = min(480,(int)ceil(xv+xr));

    const int minLevel = max(0,xminLevel);
    const int maxLevel = min(7,xmaxLevel);

    for( int in=0; in<mi_KPLeftN ;in++)
    {
        const cv::KeyPoint &xKP = mv_KPLeft[in];

        if(xKP.octave < minLevel)
            continue;
        if(xKP.octave > maxLevel)
            continue;
        if(xKP.pt.x<nMinX || xKP.pt.x>nMaxX)
            continue;
        if(xKP.pt.y<nMinY || xKP.pt.y>nMaxY)
            continue;
        vIndices.push_back(in);
    }

    return vIndices;
}




} // namespace HZH_SLAM
