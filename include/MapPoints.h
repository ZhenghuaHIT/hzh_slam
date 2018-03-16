/**
* HZH_SLAM
* MapPoints类
*/

#ifndef MAPPOINTS_H
#define MAPPOINTS_H

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

#include "KeyFrame.h"

using namespace std;

namespace HZH_SLAM
{

class KeyFrame;

class Frame;

class MapPoints
{
public:
    MapPoints(const cv::Mat &xKPw, KeyFrame *pFrame, const int &xi);

    MapPoints(const cv::Mat &xKPw, Frame* pFrame, const int &xi);

    cv::Mat m_DescriptorsKP;

    cv::Mat m_WorldPos;

    bool m_IfTemporal;

    // TrackLocalMap - UpdateLocalPoints中防止将MapPoints重复添加至mvpLocalMapPoints的标记
    long unsigned int m_LocalMPMark;
    long unsigned int m_LocalMPMark2;

    // 是否在局部地图中被删除
    bool mb_Bad;

    // 该MapPoint平均观测方向
    cv::Mat m_NormalVector;

    // 尺度预测
    float mf_MaxDistance;
    float mf_MinDistance; 
    int mi_TrackScaleLevel;

    float mf_TrackProjX;
    float mf_TrackProjY;
    float mf_TrackProjXR;
    float mf_TrackViewCos;
    bool mb_TrackInView;

    long unsigned int mi_curfId;
    static long unsigned int mi_lasfId;

    std::map<KeyFrame*,size_t> m_Observations;

    int nObs;

    int PredictScale(const float &currentDist, Frame* pF);

    bool isBad();

    cv::Mat GetWorldPos();

    float GetMinDistanceInvariance();

    float GetMaxDistanceInvariance();

    cv::Mat GetNormal();

    void AddObservation(KeyFrame* pKF, size_t idx);

    map<KeyFrame*, size_t> GetObservations();

    int GetIndexInKeyFrame(KeyFrame *pKF);

    void EraseObservation(KeyFrame* pKF);

    void SetWorldPos(const cv::Mat &Pos);

    std::mutex m_MutexPos;
    std::mutex m_MutexFeatures;
};


} // namespace hzh_slam

#endif // MAPPOINTS_H
