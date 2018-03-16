/**
* HZH_SLAM
* Frame类
*/

#ifndef FRAME_H
#define FRAME_H

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

#include "ORBextractor.h"
#include "MapPoints.h"
#include "KeyFrame.h"

using namespace std;

namespace HZH_SLAM
{

class MapPoints;

class KeyFrame;

class Frame
{
public:
    Frame();

    Frame(const Frame &frame);

    Frame(const cv::Mat &xGrayLeft, const cv::Mat &xGrayRight, const double &xstamp, ORB_SLAM2::ORBextractor* xextractorLeft, ORB_SLAM2::ORBextractor* xextractorRight,
          cv::Mat &xK, cv::Mat &xDistCoef, const float &xbf);

    //ORB特征提取器
    ORB_SLAM2::ORBextractor* mp_ORBextractorLeft;
    ORB_SLAM2::ORBextractor* mp_ORBextractorRight;

    //相机内参 畸变参数
    cv::Mat m_K;
    cv::Mat m_DistCoef;

    //图像格式
    bool mb_RGB;

    // b*f
    float mf_bf;

    // ID
    long unsigned int mi_curfId;
    static long unsigned int mi_lasfId;

    // 金字塔信息
    int mi_ScaleLevel; // 提取金字塔层数

    float mf_ScaleFactor; // 金字塔尺度因子
    vector<float> mv_ScaleFactors; // 金字塔尺度因子序列
    vector<float> mv_InvScaleFactors;// 倒数
    vector<float> mv_LevelSigma2; // 平方
    vector<float> mv_InvLevelSigma2; // 平方倒数

    // 特征点
    vector<cv::KeyPoint> mv_KPLeft;
    vector<cv::KeyPoint> mv_KPRight;
    cv::Mat m_DescriptorsLeft;
    cv::Mat m_DescriptorsRight;
    int mi_KPLeftN;

    // 双目特征匹配
    std::vector<float> mv_uRight;
    std::vector<float> mv_Depth;

    //创建该Frame对应的mappoints
    std::vector<MapPoints*>  mvp_MapPoints;
    std::vector<bool> mvb_Outlier;

    // 相机坐标
    cv::Mat m_Tcw;
    cv::Mat m_Rcw;
    cv::Mat m_Rwc; // RT
    cv::Mat m_tcw;  // t
    cv::Mat m_Ow ; // T-1 中的-RTt

    // 参考关键帧
    KeyFrame* mF_ReferenceKF;

    // 求Twc中的R,t
    void PoseMatrixInversion(const cv::Mat &xTcw);

    //提取特征的点
    void ExtractORB(int flag, const cv::Mat &im);

    // 双目重建
    void ORB_ComputeStereoMatches();
    int ORB_DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
    void HZH_ComputeStereoMatches(const cv::Mat &xGrayLeft, const cv::Mat &xGrayRight);

    // 特征点投影到世界坐标系下
    cv::Mat KPProjectWorld(const int &i);

    // 在当前帧中搜索KP
    vector<size_t> GetKPInCurF(const float &xu, const float  &xv, const float  &xr, const int &xminLevel, const int &xmaxLevel);


};





}// namespace HZH_SLAM

#endif // TRACKING_H
