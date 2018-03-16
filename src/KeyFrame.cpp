/**
* HZH_SLAM
* KF类
*/

#include"KeyFrame.h"

using namespace std;

namespace HZH_SLAM
{

long unsigned int KeyFrame::mi_lasfId=0;

KeyFrame::KeyFrame(Frame &frame):
  mvp_MapPoints(frame.mvp_MapPoints), m_DescriptorsLeft(frame.m_DescriptorsLeft),
  m_K(frame.m_K.clone()),mf_bf(frame.mf_bf),
  mi_ScaleLevel(frame.mi_ScaleLevel),mf_ScaleFactor(frame.mf_ScaleFactor),mv_ScaleFactors(frame.mv_ScaleFactors),
  mv_InvScaleFactors(frame.mv_InvScaleFactors),mv_LevelSigma2(frame.mv_LevelSigma2),mv_InvLevelSigma2(frame.mv_InvLevelSigma2),
  mv_KPLeft(frame.mv_KPLeft),mv_KPRight(frame.mv_KPRight),mi_KPLeftN(frame.mi_KPLeftN),mv_uRight(frame.mv_uRight),
  mv_Depth(frame.mv_Depth),m_Ow(frame.m_Ow),m_Tcw(frame.m_Tcw),m_posechangmark(0)
{
    mi_curfId = mi_lasfId++;
}

void KeyFrame::KFAddMapPoint(MapPoints* xpMP, const size_t &xid)
{
    mvp_MapPoints[xid]=xpMP;
}

vector<MapPoints*> KeyFrame::GetLocalMapPoints()
{
    unique_lock<mutex> lock(m_MutexFeatures);
    return mvp_MapPoints;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(m_MutexPose);
    return m_Tcw.clone();
}

void KeyFrame::EraseMapPointMatch(MapPoints* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvp_MapPoints[idx]=static_cast<MapPoints*>(NULL);
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(m_MutexPose);
    Tcw_.copyTo(m_Tcw);
    cv::Mat Rcw = m_Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = m_Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    m_Ow = -Rwc*tcw;

    m_Twc = cv::Mat::eye(4,4,m_Tcw.type());
    Rwc.copyTo(m_Twc.rowRange(0,3).colRange(0,3));
    m_Ow.copyTo(m_Twc.rowRange(0,3).col(3));

}


}// namespace Hzh_slam
