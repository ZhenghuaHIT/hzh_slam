/**
* HZH_SLAM
* Map类
*/

#ifndef MAP_H
#define MAP_H

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
#include "MapPoints.h"

using namespace std;

namespace HZH_SLAM
{

class Map
{
public:
    Map();

    std::set<MapPoints*> msp_MapPoints;
    std::vector<KeyFrame*> mvp_KeyFrames;

    void MapAddKeyFrame(KeyFrame *xpKF);

    void MapAddMapPoint(MapPoints *xpMP);

    long unsigned int KeyFramesInMap();
    std::mutex m_MutexMap;

};


} // namespace hzh_slam

#endif // MAPPOINTS_H
