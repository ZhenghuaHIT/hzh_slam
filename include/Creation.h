/**
* HZH_SLAM
* kitti数据集程序入口
*/

#ifndef CREATION_H
#define CREATION_H

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

#include "Tracking.h"
#include "Map.h"

using namespace std;

namespace HZH_SLAM
{

class Tracking;
class Map;
class LocalMap;

class Creation
{

public:
    Creation(const string &xstr_ConfigFile, const bool xb_UseViewer = false);

    void TrackStereo(const cv::Mat &mat_imLeft, const cv::Mat &mat_imRight, const double &d_stamp);

    void closeSLAM();

private:

    // Tracking 指针
    Tracking* mp_tracker;

    //LocalMap 指针
    LocalMap* mp_LocalMapper;
    std::thread* mpt_LocalMapping;

    // 创建Map
    Map* mp_Map;


};

}// namespace HZH_SLAM
#endif // CREATION_H
