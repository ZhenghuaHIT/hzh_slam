/**
* HZH_SLAM
* LocalMAP类
*/

#include"LocalMap.h"

using namespace std;

namespace HZH_SLAM
{

LocalMap::LocalMap(Map *pMap):
    mb_AbortBA(false), mp_Map(pMap), mb_AcceptKeyFrames(true),mb_FinishRequested(false),mb_Finished(true)
{}

void LocalMap::Start()
{
    mb_Finished = false;
    while(1)
    {

        // 告诉Tracking，LocalMapping正处于繁忙状态，
        // LocalMapping线程处理的关键帧都是Tracking线程发过的
        // 在LocalMapping线程还没有处理完关键帧之前Tracking线程最好不要发送太快
        // SetAcceptKeyFrames(false);
        // 等待处理的关键帧列表不为空
        if(CheckNewKeyFrames())
        {
            // 将关键帧插入地图
            ProcessNewKeyFrame();
            // 已经处理完队列中的最后的一个关键帧
            if(!CheckNewKeyFrames())
            {
                mb_AbortBA = false;
                if(mp_Map->KeyFramesInMap()>2)
                {
                    cout<<" ////———— Local BA RUNNING ————\\\\\\\\ "<<endl;
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

                    Optimization::LocalPoseOptimization(mp_LocalCurrentKF,&mb_AbortBA, mp_Map);

                    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                    cout<<"Local BA Optimization Cost Time is: "<<ttrack<<endl;
                    cout<<" \\\\\\\\———— Local BA FINISH  ————//// "<<endl;
                }
            }

        }
        else
        {
             // usleep(3000);
             std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }

        // Tracking will see that Local Mapping is not busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        //usleep(3000);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

    }

    SetFinish();
}

void LocalMap::LocalMapAddKeyFrame(KeyFrame* xpKF)
{
    unique_lock<mutex> lock(m_MutexNewKFs);
    mlp_LocalKeyFrames.push_back(xpKF);
}

bool LocalMap::AcceptKeyFrames()
{
    unique_lock<mutex> lock(m_MutexAccept);
    return mb_AcceptKeyFrames;
}

void LocalMap::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(m_MutexAccept);
    mb_AcceptKeyFrames=flag;
}

void LocalMap::InterruptBA()
{
    mb_AbortBA = true;
}

bool LocalMap::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(m_MutexNewKFs);
    return(!mlp_LocalKeyFrames.empty());
}

void LocalMap::ProcessNewKeyFrame()
{
    // 步骤1：从缓冲队列中取出一帧关键帧
    // Tracking线程向LocalMapping中插入关键帧存在该队列中
    {
        unique_lock<mutex> lock(m_MutexNewKFs);
        // 从列表中获得一个等待被插入的关键帧
        mp_LocalCurrentKF = mlp_LocalKeyFrames.front();
        mlp_LocalKeyFrames.pop_front();
    }

    // 步骤5：将该关键帧插入到地图中
    if(mp_LocalCurrentKF->mi_curfId!=0)
        mp_Map->MapAddKeyFrame(mp_LocalCurrentKF);

}

void LocalMap::RequestFinish()
{
    unique_lock<mutex> lock(m_MutexFinish);
    mb_FinishRequested = true;
}

bool LocalMap::CheckFinish()
{
    unique_lock<mutex> lock(m_MutexFinish);
    return mb_FinishRequested;
}

void LocalMap::SetFinish()
{
    unique_lock<mutex> lock(m_MutexFinish);
    mb_Finished = true;
}

bool LocalMap::isFinished()
{
    unique_lock<mutex> lock(m_MutexFinish);
    return mb_Finished;
}

}// namespace Hzh_slam
