/**
* HZH_SLAM
* Registration类
*/

#ifndef REGISTRATION_H
#define REGISTRATION_H

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

class Registration
{
public:
    Registration();

    int FramesProjection(Frame &xCurrentFrame, const Frame &xLastFrame );

    int FramesProjection(Frame &xCurrentFrame, const vector<MapPoints*> &xvpMapPoints);

    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    float RadiusByViewingCos(const float &viewCos);

};


} // namespace hzh_slam

#endif // REGISTRATION_H
