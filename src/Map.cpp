/**
* HZH_SLAM
* MAP类
*/

#include"Map.h"

using namespace std;

namespace HZH_SLAM
{

Map::Map()
{}

void Map::MapAddKeyFrame(KeyFrame *xpKF)
{
    mvp_KeyFrames.push_back(xpKF);
}


void Map::MapAddMapPoint(MapPoints *xpMP)
{
    msp_MapPoints.insert(xpMP);
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(m_MutexMap);
    return mvp_KeyFrames.size();
}

}// namespace Hzh_slam
