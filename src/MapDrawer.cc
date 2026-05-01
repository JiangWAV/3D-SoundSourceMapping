/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <future>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

        mViewerDenseMappingDistMax = fSettings["Viewer.DenseMappingDistMax"];
        mfSSLineDist = fSettings["Viewer.SSLineDist"];
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

// void MapDrawer::DrawPointCloud(){
// 	bool bDrawPointCloud = true;
//     const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
//     if(bDrawPointCloud)
//     {
//         for(size_t i=0; i<vpKFs.size(); i++)
//         {
//             KeyFrame* pKF = vpKFs[i];
//             cv::Mat Twc = pKF->GetPoseInverse().t();

//             glPushMatrix();
//             glMultMatrixf(Twc.ptr<GLfloat>(0));

// 			glPointSize(mPointSize*1.0);
// 			glBegin(GL_POINTS);

// 			cv::Mat imRGB;
// 			pKF->mPCImRGBRaw.copyTo(imRGB);
// 			// cvtColor(imRGB,imRGB,CV_BayerGB2RGB);

// 			cv::Mat imDepth;
// 			pKF->mPCImDepth.copyTo(imDepth);

// 			static float invfx = pKF->mPCinvfx;
//     		static float invfy = pKF->mPCinvfy;
// 			static float cx = pKF->mPCcx;
//     		static float cy = pKF->mPCcy;

// 			for(int uIndex=0; uIndex<imDepth.cols; uIndex++){
// 				for(int vIndex=0; vIndex<imDepth.rows; vIndex++){
		
// 					const float v = float(vIndex);
// 					const float u = float(uIndex);

// 					const float z = imDepth.at<float>(int(v),int(u));

// 					const float x = (u-cx)*z*invfx;
// 					const float y = (v-cy)*z*invfy;

// 					if(z>0 && sqrt(pow(x,2)+pow(y,2)+pow(z,2))<mViewerDenseMappingDistMax){
// 						// Undistort corners
// 						//mat=mat.reshape(2);
// 						//cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
// 						//mat=mat.reshape(1);
// 						cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

// 						Eigen::Matrix<double,6,1> pPoint;

// 						pPoint << x3Dc.at<float>(0,0), x3Dc.at<float>(1,0), x3Dc.at<float>(2,0), imRGB.at<cv::Vec3b>(int(v),int(u))[0], imRGB.at<cv::Vec3b>(int(v),int(u))[1], imRGB.at<cv::Vec3b>(int(v),int(u))[2];

// 						glColor3f(double(pPoint(3,0)/255.0),double(pPoint(4,0)/255.0),double(pPoint(5,0)/255.0));
// 						glVertex3f(pPoint(0,0),pPoint(1,0),pPoint(2,0));
// 					}
// 				}
// 			}
// 			glEnd();
//             glPopMatrix();
//         }
//     }
// }


// void MapDrawer::DrawPointCloud(){
// 	bool bDrawPointCloud = true;
//     const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
//     if(bDrawPointCloud)
//     {
//         int step = 2; // 采样步长
//         int N = 100;   // 只显示最近10个关键帧

//         for(int i = max(0,(int)vpKFs.size()-N); i<vpKFs.size(); i++)
//         {
//             KeyFrame* pKF = vpKFs[i];
//             cv::Mat Twc = pKF->GetPoseInverse().t();

//             glPushMatrix();
//             glMultMatrixf(Twc.ptr<GLfloat>(0));

//             glPointSize(mPointSize*1.0);
//             glBegin(GL_POINTS);

//             cv::Mat imRGB = pKF->mPCImRGBRaw;
//             cv::Mat imDepth = pKF->mPCImDepth;

//             float invfx = pKF->mPCinvfx;
//             float invfy = pKF->mPCinvfy;
//             float cx = pKF->mPCcx;
//             float cy = pKF->mPCcy;
//             float maxDist2 = mViewerDenseMappingDistMax * mViewerDenseMappingDistMax;

//             for(int u=0; u<imDepth.cols; u+=step){
//                 for(int v=0; v<imDepth.rows; v+=step){
//                     float z = imDepth.at<float>(v,u);
//                     if(z<=0) continue;

//                     float x = (u-cx)*z*invfx;
//                     float y = (v-cy)*z*invfy;
//                     float dist2 = x*x + y*y + z*z;

//                     if(dist2 < maxDist2){
//                         cv::Vec3b color = imRGB.at<cv::Vec3b>(v,u);
//                         glColor3f(color[2]/255.0, color[1]/255.0, color[0]/255.0);
//                         glVertex3f(x,y,z);
//                     }
//                 }
//             }
//             glEnd();
//             glPopMatrix();
//         }
//     }
// }

void MapDrawer::DrawPointCloud(){
    bool bDrawPointCloud = true;
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawPointCloud)
    {
        int N = 100;   // 只显示最近10个关键帧
        int step = 1; // 采样步长

        for(int i = max(0,(int)vpKFs.size()-N); i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glPointSize(mPointSize*1.0);

            cv::Mat imRGB = pKF->mPCImRGBRaw;
            cv::Mat imDepth = pKF->mPCImDepth;

            float invfx = pKF->mPCinvfx;
            float invfy = pKF->mPCinvfy;
            float cx = pKF->mPCcx;
            float cy = pKF->mPCcy;
            float maxDist2 = mViewerDenseMappingDistMax * mViewerDenseMappingDistMax;

            // ---- 定义任务函数 ----
            auto worker = [&](int v_start, int v_end){
                std::vector<PointXYZRGB> localBuf;
                localBuf.reserve((v_end-v_start) * imDepth.cols / (step*step));

                for(int v=v_start; v<v_end; v+=step){
                    for(int u=0; u<imDepth.cols; u+=step){
                        float z = imDepth.at<float>(v,u);
                        if(z <= 0) continue;

                        float x = (u-cx)*z*invfx;
                        float y = (v-cy)*z*invfy;
                        float dist2 = x*x + y*y + z*z;

                        if(dist2 < maxDist2){
                            cv::Vec3b color = imRGB.at<cv::Vec3b>(v,u);
                            PointXYZRGB p{x,y,z,(uchar)color[2],(uchar)color[1],(uchar)color[0]};
                            localBuf.push_back(p);
                        }
                    }
                }
                return localBuf;
            };

            // ---- 启动并行任务（分4块） ----
            int h = imDepth.rows;
            auto f1 = std::async(std::launch::async, worker, 0, h/4);
            auto f2 = std::async(std::launch::async, worker, h/4, h/2);
            auto f3 = std::async(std::launch::async, worker, h/2, 3*h/4);
            auto f4 = std::async(std::launch::async, worker, 3*h/4, h);

            // ---- 合并结果 ----
            std::vector<PointXYZRGB> buffer;
            buffer.reserve(imDepth.rows * imDepth.cols / (step*step));

            for(auto& f : {&f1,&f2,&f3,&f4}){
                auto res = f->get();
                buffer.insert(buffer.end(), res.begin(), res.end());
            }

            // ---- 渲染 ----
            glBegin(GL_POINTS);
            for(auto &p : buffer){
                glColor3f(p.r/255.0f, p.g/255.0f, p.b/255.0f);
                glVertex3f(p.x, p.y, p.z);
            }
            glEnd();

            glPopMatrix();
        }
    }
}

// ----------- 绘制声源位置 -------------
void MapDrawer::DrawSoundSource(const Eigen::MatrixXd& SoundSource){
    Eigen::MatrixXd SoundSourceStateGlobalXYZ=SoundSource;
    // SoundSourceStateGlobalXYZ(0,0) = 1;
    // SoundSourceStateGlobalXYZ(1,0) = 0.5;
    // SoundSourceStateGlobalXYZ(2,0) = 0.5;
    if(SoundSourceStateGlobalXYZ.size() == 3) // 确保坐标有效
    {
        float cx = SoundSourceStateGlobalXYZ(0,0);
        float cy = SoundSourceStateGlobalXYZ(1,0);
        float cz = SoundSourceStateGlobalXYZ(2,0);
        float d  = mfSSLineDist;  // 半边长

        // 画立方体
        glPushMatrix();
        glColor3f(1.0, 1.0, 0.0); // 黄色
        // 前后左右上下六个面
        glBegin(GL_QUADS);
        // 前面
        glVertex3f(cx-d, cy-d, cz-d);
        glVertex3f(cx+d, cy-d, cz-d);
        glVertex3f(cx+d, cy+d, cz-d);
        glVertex3f(cx-d, cy+d, cz-d);
        // 后面
        glVertex3f(cx-d, cy-d, cz+d);
        glVertex3f(cx+d, cy-d, cz+d);
        glVertex3f(cx+d, cy+d, cz+d);
        glVertex3f(cx-d, cy+d, cz+d);
        // 左面
        glVertex3f(cx-d, cy-d, cz-d);
        glVertex3f(cx-d, cy-d, cz+d);
        glVertex3f(cx-d, cy+d, cz+d);
        glVertex3f(cx-d, cy+d, cz-d);
        // 右面
        glVertex3f(cx+d, cy-d, cz-d);
        glVertex3f(cx+d, cy-d, cz+d);
        glVertex3f(cx+d, cy+d, cz+d);
        glVertex3f(cx+d, cy+d, cz-d);
        // 上面
        glVertex3f(cx-d, cy+d, cz-d);
        glVertex3f(cx+d, cy+d, cz-d);
        glVertex3f(cx+d, cy+d, cz+d);
        glVertex3f(cx-d, cy+d, cz+d);
        // 下面
        glVertex3f(cx-d, cy-d, cz-d);
        glVertex3f(cx+d, cy-d, cz-d);
        glVertex3f(cx+d, cy-d, cz+d);
        glVertex3f(cx-d, cy-d, cz+d);
        glEnd();

        // 画边框
        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,0.0f,0.0f);
        glBegin(GL_LINES);
        // 立方体的12条边
        glVertex3f(cx-d,cy-d,cz-d); glVertex3f(cx+d,cy-d,cz-d);
        glVertex3f(cx+d,cy-d,cz-d); glVertex3f(cx+d,cy+d,cz-d);
        glVertex3f(cx+d,cy+d,cz-d); glVertex3f(cx-d,cy+d,cz-d);
        glVertex3f(cx-d,cy+d,cz-d); glVertex3f(cx-d,cy-d,cz-d);

        glVertex3f(cx-d,cy-d,cz+d); glVertex3f(cx+d,cy-d,cz+d);
        glVertex3f(cx+d,cy-d,cz+d); glVertex3f(cx+d,cy+d,cz+d);
        glVertex3f(cx+d,cy+d,cz+d); glVertex3f(cx-d,cy+d,cz+d);
        glVertex3f(cx-d,cy+d,cz+d); glVertex3f(cx-d,cy-d,cz+d);

        glVertex3f(cx-d,cy-d,cz-d); glVertex3f(cx-d,cy-d,cz+d);
        glVertex3f(cx+d,cy-d,cz-d); glVertex3f(cx+d,cy-d,cz+d);
        glVertex3f(cx+d,cy+d,cz-d); glVertex3f(cx+d,cy+d,cz+d);
        glVertex3f(cx-d,cy+d,cz-d); glVertex3f(cx-d,cy+d,cz+d);
        glEnd();
        glPopMatrix();
    }




}
} //namespace ORB_SLAM
