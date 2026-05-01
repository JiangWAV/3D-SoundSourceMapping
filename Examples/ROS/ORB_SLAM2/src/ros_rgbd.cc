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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point.h>

    #include <geometry_msgs/PoseStamped.h>
    #include <tf2/LinearMath/Quaternion.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    #include <tf2_ros/transform_broadcaster.h>


#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle& nh):mpSLAM(pSLAM)
        {
            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera_pose", 10);
        }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

        void SendSSP(const geometry_msgs::Point::ConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
        ros::Publisher pose_pub;
        tf2_ros::TransformBroadcaster tf_br;
};

        geometry_msgs::PoseStamped cvMatToPoseMsg(const cv::Mat &Tcw, double timestamp)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time(timestamp);
        pose_msg.header.frame_id = "map";   // 世界坐标系

        // 提取旋转和平移
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);

        // 转换成 Eigen 再转四元数
        cv::Mat Rwc = Rcw.t();
        cv::Mat twc = -Rwc * tcw;

        Eigen::Matrix3f eigR;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                eigR(i,j) = Rwc.at<float>(i,j);

        Eigen::Quaternionf q(eigR);

        pose_msg.pose.position.x = twc.at<float>(0);
        pose_msg.pose.position.y = twc.at<float>(1);
        pose_msg.pose.position.z = twc.at<float>(2);

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        return pose_msg;
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;

        ImageGrabber igb(&SLAM, nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

        // 订阅声源估计
        ros::Subscriber sub = nh.subscribe("/sound_source_position", 1, &ImageGrabber::SendSSP,&igb);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}



void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,  const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

        cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

       if(!Tcw.empty())
        {
            double timestamp = cv_ptrRGB->header.stamp.toSec();

            geometry_msgs::PoseStamped pose_msg = cvMatToPoseMsg(Tcw, timestamp);
            pose_pub.publish(pose_msg);

            // 发布 TF（同理）
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time(timestamp);
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "camera_link";

            transformStamped.transform.translation.x = pose_msg.pose.position.x;
            transformStamped.transform.translation.y = pose_msg.pose.position.y;
            transformStamped.transform.translation.z = pose_msg.pose.position.z;
            transformStamped.transform.rotation = pose_msg.pose.orientation;

            tf_br.sendTransform(transformStamped);
        }
}

    void ImageGrabber::SendSSP(const geometry_msgs::Point::ConstPtr& msg)
    {
        // 将 ROS Point 转成 Eigen::MatrixXd
        Eigen::MatrixXd SoundSource(3,1);
        SoundSource(0,0) = msg->x;
        SoundSource(1,0) = msg->y;
        SoundSource(2,0) = msg->z;

        // 调用 System 的函数
        mpSLAM->SSP(SoundSource);
    }
