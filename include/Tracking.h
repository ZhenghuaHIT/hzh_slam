/**
* HZH_SLAM
* Tracking类
*/

#ifndef TRACKING_H
#define TRACKING_H

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<cmath>
#include<mutex>
#include<string>
#include<thread>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Creation.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoints.h"
#include "LocalMap.h"
#include "Registration.h"
#include "Optimization.h"

using namespace std;

namespace HZH_SLAM
{

class Tracking
{
public:
    Tracking( const string &xstr_ConfigFile, Map* xpMap );

    // 左右图像
    cv::Mat mMat_GrayLeft;
    cv::Mat mMat_GrayRight;

    //相机内参 畸变参数
    cv::Mat m_K;
    cv::Mat m_DistCoef;

    //图像格式
    bool mb_RGB;

    // b*f
    float mf_bf;

    // fps关键帧创建规则
    int mi_MinFrames;
    int mi_MaxFrames;

    float m_ThDepth;

    //ORB特征提取器
    ORB_SLAM2::ORBextractor* mp_ORBextractorLeft;
    ORB_SLAM2::ORBextractor* mp_ORBextractorRight;

    //当前帧与上一帧
    Frame mF_CurrentFrame;
    Frame mF_LastFrame;

    //两帧的运动速度
    cv::Mat m_Velocity;

    //上一关键帧与上一关键帧ID
    unsigned int mi_LastKeyFrameId;
    KeyFrame* mF_LastKeyFrame ;

    // 局部地图
    LocalMap* mp_LocalMapper;
    KeyFrame* mF_ReferenceKF;
    std::vector<KeyFrame*> mvp_LocalKeyFrames;
    std::vector<MapPoints*> mvp_LocalMapPoints;

    //Current matches in frame
    int mi_FrameInliers;

    //系统状态
    enum Systemstate{
        SYSTEM_STANDBY=0,
        SYSTEM_NOT_INITIALIZED=1,
        SYSTEM_TRACKING_ON=2,
        SYSTEM_LOST=3
    };

    Systemstate me_state;

    //Map 包含所有KF与KeyPoints
    Map* mp_Map;

    // 临时特征点
    list<MapPoints*> mlp_TemporalPoints;

    void StereoTrack(const cv::Mat &mat_imLeft, const cv::Mat &mat_imRight, const double &d_stamp);

    void TrackCloseFrame();

    bool TrackWithMotionModel();

    void SystemInitialization();

    void SetLocalMapper(LocalMap* pLocalMapper);

    void AddLastFrameTMP();

    bool NeedNewKeyFrame();

    void CreateNewKeyFrame();

    bool TrackLocalMap();

    void UpdateLocalKeyFrames();

    void UpdateLocalPoints();

    void ProjectLocalPoints();

    bool TrackWithReferenceFrame();

};

}// namespace HZH_SLAM

#endif // TRACKING_H
