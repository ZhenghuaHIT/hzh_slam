/**
* HZH_SLAM
* 系统创建类
*/

#include "Creation.h"

using namespace std;

namespace HZH_SLAM
{

Creation::Creation(const string &xstr_ConfigFile, const bool xb_UseViewer)
{
    // 输出欢迎信息
    cout<< endl <<
    "HZH_SLAM. "<< endl <<
    "It's a Stereo SLAM that references the ORB_SLAM2. "<< endl <<
    "The system doesn't have loopclosing."<< endl <<
    "Zhenghua Hou, Harbin Institute of Technology. "<< endl;

    //检查是否含有配置文件
    cv::FileStorage fsSettings(xstr_ConfigFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open Config file at: " << xstr_ConfigFile << endl;
       exit(-1);
    }

    // 创建Map
    mp_Map = new Map();
    // 创建可视化

    // 初始化Tracking线程
    mp_tracker = new Tracking(xstr_ConfigFile, mp_Map);

    // 初始化LoaclMapping线程
    mp_LocalMapper = new LocalMap(mp_Map);
    mpt_LocalMapping = new thread(&HZH_SLAM::LocalMap::Start,mp_LocalMapper);
    mp_tracker->SetLocalMapper(mp_LocalMapper); //Localmap指向track

    // 初始化Viewer线程


}

void Creation::closeSLAM()
{
    mp_LocalMapper->RequestFinish();
    // Wait until all thread have effectively stopped
    while(!mp_LocalMapper->isFinished())
    {
        //usleep(5000);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    //usleep(5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    cout<<"HZH_SLAM is closed!"<<endl << endl;
}

void Creation::TrackStereo(const cv::Mat &mat_imLeft, const cv::Mat &mat_imRight, const double &d_stamp)
{
    mp_tracker->StereoTrack(mat_imLeft, mat_imRight, d_stamp);

}


} //namespace HZH_SLAM
