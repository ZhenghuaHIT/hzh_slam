/**
* HZH_SLAM
* KF类
*/

#ifndef KEYFRAME_H
#define KEYFRAME_H

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

#include "Frame.h"

using namespace std;

namespace HZH_SLAM
{

class Frame;
class MapPoints;

class KeyFrame
{
public:
    KeyFrame(Frame &frame);

    // ID
    long unsigned int mi_curfId;
    static long unsigned int mi_lasfId;

    vector<MapPoints*> mvp_MapPoints;

    cv::Mat m_DescriptorsLeft;

    cv::Mat m_K;
    float mf_bf;
    int mi_ScaleLevel;
    float mf_ScaleFactor;
    vector<float> mv_ScaleFactors;
    vector<float> mv_InvScaleFactors;
    vector<float> mv_LevelSigma2;
    vector<float> mv_InvLevelSigma2;
    vector<cv::KeyPoint> mv_KPLeft;
    vector<cv::KeyPoint> mv_KPRight;
    int mi_KPLeftN;
    vector<float> mv_uRight;
    vector<float> mv_Depth;
    cv::Mat m_Ow;
    cv::Mat m_Tcw;
    cv::Mat m_Twc;

    // 改变
    int m_posechangmark;

    void KFAddMapPoint(MapPoints* xpMP, const size_t &xid);

    vector<MapPoints*> GetLocalMapPoints();

    cv::Mat GetPose();

    void EraseMapPointMatch(MapPoints* pMP);

    void SetPose(const cv::Mat &Tcw_);


    std::mutex m_MutexFeatures;
    std::mutex m_MutexPose;


};


} // namespace hzh_slam

#endif // KEYFRAME_H
