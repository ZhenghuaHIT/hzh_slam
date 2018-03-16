/**
* HZH_SLAM
* MapPoints类
*/

#include"MapPoints.h"

using namespace std;

namespace HZH_SLAM
{
long unsigned int MapPoints::mi_lasfId=0;

MapPoints::MapPoints(const cv::Mat &xKPw, KeyFrame *pFrame, const int &xi):
    m_IfTemporal(false),m_LocalMPMark(0),m_LocalMPMark2(0),mb_Bad(false),mf_MaxDistance(0),nObs(0)
{

    xKPw.copyTo(m_WorldPos);

    m_DescriptorsKP = pFrame->m_DescriptorsLeft.row(xi).clone();

    cv::Mat Ow = pFrame->m_Ow;
    m_NormalVector = m_WorldPos - Ow;// 世界坐标系下相机到3D点的向量
    m_NormalVector = m_NormalVector/cv::norm(m_NormalVector);// 世界坐标系下相机到3D点的单位向量

    cv::Mat PC = xKPw - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mv_KPLeft[xi].octave;
    const float levelScaleFactor =  pFrame->mv_ScaleFactors[level];
    const int nLevels = pFrame->mi_ScaleLevel;

    mf_MaxDistance = dist*levelScaleFactor;
    mf_MinDistance = mf_MaxDistance/pFrame->mv_ScaleFactors[nLevels-1];

    mi_curfId = mi_lasfId++;
}

MapPoints::MapPoints(const cv::Mat &xKPw, Frame* pFrame, const int &xi):
    m_IfTemporal(false),m_LocalMPMark(0),m_LocalMPMark2(0),mb_Bad(false),mf_MaxDistance(0),nObs(0)
{
    xKPw.copyTo(m_WorldPos);

    m_DescriptorsKP = pFrame->m_DescriptorsLeft.row(xi).clone();

    mi_curfId = mi_lasfId++;
}

bool MapPoints::isBad()
{
    unique_lock<mutex> lock(m_MutexFeatures);
    unique_lock<mutex> lock2(m_MutexPos);
    return mb_Bad;
}

int MapPoints::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(m_MutexPos);
        ratio = mf_MaxDistance/currentDist;
    }
    float mf_LogScaleFactor = log(pF->mf_ScaleFactor);
    int nScale = ceil(log(ratio)/mf_LogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mi_ScaleLevel)
        nScale = pF->mi_ScaleLevel-1;
    return nScale;
}

cv::Mat MapPoints::GetWorldPos()
{
    unique_lock<mutex> lock(m_MutexPos);
    return m_WorldPos.clone();
}


float MapPoints::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(m_MutexPos);
    return 0.8f*mf_MinDistance;
}

float MapPoints::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(m_MutexPos);
    return 1.2f*mf_MaxDistance;
}

cv::Mat MapPoints::GetNormal()
{
    unique_lock<mutex> lock(m_MutexPos);
    return m_NormalVector.clone();
}


void MapPoints::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(m_MutexFeatures);
    if(m_Observations.count(pKF))
        return; // 查找是否有pKF
    // 记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
    m_Observations[pKF]=idx;

    if(pKF->mv_uRight[idx]>=0)
        nObs+=2; // 双目或者grbd

}

map<KeyFrame*, size_t> MapPoints::GetObservations()
{
    unique_lock<mutex> lock(m_MutexFeatures);
    return m_Observations;
}

int MapPoints::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(m_MutexFeatures);
    if(m_Observations.count(pKF))
        return m_Observations[pKF];
    else
        return -1;
}

void MapPoints::EraseObservation(KeyFrame* pKF)
{
    {
        unique_lock<mutex> lock(m_MutexFeatures);
        if(m_Observations.count(pKF))
        {
            int idx = m_Observations[pKF];
            if(pKF->mv_uRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            m_Observations.erase(pKF);
        }
    }

}

void MapPoints::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock(m_MutexPos);
    Pos.copyTo(m_WorldPos);
}


}// namespace Hzh_slam
