/**
* HZH_SLAM
* Registration类
*/

#include"Registration.h"

using namespace std;

namespace HZH_SLAM
{

Registration::Registration()
{}

int Registration::FramesProjection(Frame &xCurrentFrame, const Frame &xLastFrame )
{
    int nmatches = 0;

    const cv::Mat Rcw = xCurrentFrame.m_Tcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = xCurrentFrame.m_Tcw.rowRange(0,3).col(3);

    for(int i=0; i<xLastFrame.mi_KPLeftN; i++)
    {
        MapPoints* pMP = xLastFrame.mvp_MapPoints[i];

        if(pMP)
        {
            if(!xLastFrame.mvb_Outlier[i])
            {

                //cout<<"123"<<pMP->m_WorldPos<<endl;
                // 对上一帧有效的MapPoints进行跟踪
                cv::Mat MPw = pMP->m_WorldPos;
                //cout<<"1234"<<MPw<<endl;
                cv::Mat MPc = Rcw*MPw+tcw; //投影到当前帧


                const float xc = MPc.at<float>(0);
                const float yc = MPc.at<float>(1);
                const float invzc = 1.0/MPc.at<float>(2);

                if(invzc<0)
                    continue;

                float u = xCurrentFrame.m_K.at<float>(0,0)*xc*invzc+xCurrentFrame.m_K.at<float>(0,2);
                float v = xCurrentFrame.m_K.at<float>(1,1)*yc*invzc+xCurrentFrame.m_K.at<float>(1,2);

                if(u<0 || u>640)
                    continue;
                if(v<0 || v>480)
                    continue;

                int nLastOctave = xLastFrame.mv_KPLeft[i].octave;

                // Search in a window. Size depends on scale
                float th = 20.0;
                float radius = th*xCurrentFrame.mv_ScaleFactors[nLastOctave]; // 尺度越大，搜索范围越大

                vector<size_t> vIndices2;

                 // 在[nLastOctave-1, nLastOctave+1]中搜索
                vIndices2 = xCurrentFrame.GetKPInCurF(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->m_DescriptorsKP; // 描述子为创建MP的描述子

                int bestDist = 256;
                int bestIdx2 = -1;

                // 遍历满足条件的特征点
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;

                    if(xCurrentFrame.mv_uRight[i2]>0)
                    {
                        // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                        const float ur = u - xCurrentFrame.mf_bf*invzc;
                        const float er = fabs(ur - xCurrentFrame.mv_uRight[i2]);
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &dC = xCurrentFrame.m_DescriptorsLeft.row(i2);

                    const int dist = DescriptorDistance(dMP,dC);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=100)
                {
                    xCurrentFrame.mvp_MapPoints[bestIdx2]=pMP; // 为当前帧添加MapPoint
                    nmatches++;    
                }
            }
        }
    }

    return nmatches;
}

int Registration::FramesProjection(Frame &xCurrentFrame, const vector<MapPoints*> &xvpMapPoints)
{
    int nmatches=0;

    for(size_t iMP=0; iMP<xvpMapPoints.size(); iMP++)
    {
        MapPoints* pMP = xvpMapPoints[iMP];

        // 判断该点是否要投影
        if(!pMP->mb_TrackInView)
            continue;

        if(pMP->isBad())
            continue;

        // 通过距离预测的金字塔层数，该层数相对于当前的帧
        const int nPredictedLevel = pMP->mi_TrackScaleLevel;

        if(nPredictedLevel>7 || nPredictedLevel<0)
            continue;

        // 搜索窗口的大小取决于视角, 若当前视角和平均视角夹角接近0度时, r取一个较小的值
        float r = RadiusByViewingCos(pMP->mf_TrackViewCos);
        const float th = 4.0;
        const float thx = r*th;
        float radius = thx*xCurrentFrame.mv_ScaleFactors[nPredictedLevel]; // 尺度越大，搜索范围越大

        // 通过投影点投影到当前帧以及搜索窗口和预测的尺度进行搜索, 找出附近的兴趣点
        const vector<size_t> vIndices2 =
                xCurrentFrame.GetKPInCurF(pMP->mf_TrackProjX,pMP->mf_TrackProjY,radius,nPredictedLevel-1,nPredictedLevel+1);

        if(vIndices2.empty())
            continue;

        const cv::Mat dMP = pMP->m_DescriptorsKP; // 描述子为创建MP的描述子

        int bestDist = 256;
        int bestIdx2 = -1;

        // 遍历满足条件的特征点
        for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
        {
            const size_t i2 = *vit;

            if(xCurrentFrame.mv_uRight[i2]>0)
            {
                // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                const float ur = pMP->mf_TrackProjXR;
                const float er = fabs(ur - xCurrentFrame.mv_uRight[i2]);
                if(er>radius)
                    continue;
            }

            const cv::Mat &dC = xCurrentFrame.m_DescriptorsLeft.row(i2);

            const int dist = DescriptorDistance(dMP,dC);

            if(dist<bestDist)
            {
                bestDist=dist;
                bestIdx2=i2;
            }
        }

        if(bestDist<=100)
        {
            xCurrentFrame.mvp_MapPoints[bestIdx2]=pMP; // 为当前帧添加MapPoint
            nmatches++;
        }

    }

    cout<<"Track LocalMap matches is: "<<nmatches<<endl;
    return nmatches;


}

int Registration::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

float Registration::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}


}// namespace Hzh_slam
