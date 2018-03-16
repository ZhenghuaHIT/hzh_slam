/**
* HZH_SLAM
* kitti数据集程序入口
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<cmath>
#include<mutex>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Creation.h"
#include"Tracking.h"

using namespace std;

void LoadImage(const string &str_DatasetPath, vector<string> &vstr_ImageLeft,
          vector<string> &vstr_ImageRight, vector<double> &vd_Timestamp);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./hzhslam_kitti path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // 读取图像序列
    vector<string> vstr_LeftImage;
    vector<string> vstr_RightImage;
    vector<double> vd_TimeStamp;
    LoadImage(string(argv[2]), vstr_LeftImage, vstr_RightImage, vd_TimeStamp);

    // 创建SLAM系统
    HZH_SLAM::Creation SLAM(argv[1], false);

    // 主循环
    int i_nImages = vstr_LeftImage.size();
    for (int i=0; i<i_nImages; i++)
    {
        cv::Mat mat_imLeft, mat_imRight;
        double d_stamp;
        mat_imLeft = cv::imread(vstr_LeftImage[i],CV_LOAD_IMAGE_UNCHANGED);
        mat_imRight = cv::imread(vstr_RightImage[i],CV_LOAD_IMAGE_UNCHANGED);
        d_stamp = vd_TimeStamp[i];

        // 进入Tracking主线程,从Creation跳转
        SLAM.TrackStereo(mat_imLeft, mat_imRight, d_stamp);

    }

     SLAM.closeSLAM();
}

/**
 * @brief 从数据集中读取左右图像名称序列与时间戳序列
 *
 *
 * @param   strDatasetPath    数据集地址
 *          vstrImageLeft     左图像序列
 *          vstrImageRight    右图像序列
 *          vdTimestamp       时间戳序列
 */
void LoadImage(const string &str_DatasetPath, vector<string> &vstr_ImageLeft,
          vector<string> &vstr_ImageRight, vector<double> &vd_Timestamp)
{
    fstream datafstream;
    datafstream.open(str_DatasetPath+"/times.txt");
    string s;
    while( getline(datafstream,s) )
    {
        double ss = atof(s.c_str());
        vd_Timestamp.push_back(ss);
    }

    string str_ImageLeft, str_ImageRight;
    str_ImageLeft = str_DatasetPath + "/image_0/";
    str_ImageRight = str_DatasetPath + "/image_1/";

    int i_stampsize = vd_Timestamp.size();
    vstr_ImageLeft.resize(i_stampsize);
    vstr_ImageRight.resize(i_stampsize);

    for(int i=0; i<i_stampsize; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstr_ImageLeft[i] = str_ImageLeft + ss.str() + ".png";
        vstr_ImageRight[i] = str_ImageRight + ss.str() + ".png";
    }
}
