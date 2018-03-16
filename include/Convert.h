/**
* HZH_SLAM
* Map类
*/

#ifndef CONVERT_H
#define CONVERT_H

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

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;

namespace HZH_SLAM
{

class Convert
{
public:
    Convert();

    static g2o::SE3Quat CvMat2SE3Quat(const cv::Mat &xT);

    static cv::Mat SE3Quat2CvMat(const g2o::SE3Quat &SE3);

    static cv::Mat Eigen2CvMat(const Eigen::Matrix<double,4,4> &m);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

};


} // namespace hzh_slam

#endif // CONVERT_H
