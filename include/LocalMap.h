/**
* HZH_SLAM
* LocalMap类
*/

#ifndef LOCALMAP_H
#define LOCALMAP_H

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<cmath>
#include<mutex>
#include<string>
#include<thread>
#include<chrono>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "KeyFrame.h"
#include "MapPoints.h"
#include "Map.h"
#include "Optimization.h"

using namespace std;

namespace HZH_SLAM
{

class LocalMap
{
public:
    LocalMap(Map *pMap);


    bool AcceptKeyFrames();

    void SetAcceptKeyFrames(bool flag);

    void InterruptBA();

    void LocalMapAddKeyFrame(KeyFrame* xpKF);

    void Start();

    void RequestFinish();

    bool CheckFinish();

    void SetFinish();

    bool isFinished();

protected:

    std::list<KeyFrame*> mlp_LocalKeyFrames; //其他线程调用
    std::mutex m_MutexNewKFs;

    bool mb_AbortBA; //其他线程调用 无锁

    Map* mp_Map;

    bool mb_AcceptKeyFrames; //其他线程调用
    std::mutex m_MutexAccept;

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();

    KeyFrame* mp_LocalCurrentKF;

    std::list<MapPoints*> mlp_LocalMapPoints;

    bool mb_FinishRequested;
    std::mutex m_MutexFinish;
    bool mb_Finished;


};


} // namespace hzh_slam

#endif // LOCALMAPPOINTS_H
