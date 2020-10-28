//
// Created by qiaochunxiao on 2020/8/4.
//

//
// Created by qiao on 20-7-2.
//
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include "mynteye/logger.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include "mynteye/logger.h"
#include "mynteye/api/api.h"

#include "util_cv.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <chrono>

#include <thread>
MYNTEYE_USE_NAMESPACE
sensor_msgs::Imu        imu_msg;
void imu_ros_pub();
int main(int argc, char *argv[]) {
    clock_t start,end;
    ros::init (argc, argv, "public_ros");
    ros::NodeHandle nh;
    ros::Publisher img_pub_left = nh.advertise<sensor_msgs::Image> ("/cam0/image_raw", 1);
    ros::Publisher img_pub_right = nh.advertise<sensor_msgs::Image> ("img_out/right", 1);
    ros::Publisher imu_pub      =nh.advertise<sensor_msgs::Imu>("/imu0",1);
    ros::Publisher img_pub_left_and_right = nh.advertise<sensor_msgs::Image> ("img_out/left_and_right", 1);
    sensor_msgs::ImagePtr img_msg_left;
    sensor_msgs::ImagePtr img_msg_right;
    sensor_msgs::ImagePtr img_msg_left_and_right;
    int gravity_=9.8;

    auto &&api = API::Create(argc, argv);
    if (!api) return 1;
    bool ok;
    auto &&request = api->SelectStreamRequest(&ok);
    if (!ok) return 1;
    api->ConfigStreamRequest(request);
//    api->SetOptionValue(Option::IR_CONTROL, 80);
    // Get motion data from callback
    std::atomic_uint imu_count(0);
    std::shared_ptr<mynteye::ImuData> imu;
    std::mutex imu_mtx;
    api->SetMotionCallback(
            [&imu_count, &imu, &imu_mtx](const api::MotionData &data) {
                CHECK_NOTNULL(data.imu);
                ++imu_count;
                {
                    std::lock_guard<std::mutex> _(imu_mtx);
                    imu = data.imu;
                }
//                imu_msg.header.frame_id="odom";
//                imu_msg.header.stamp.nsec=imu->timestamp%1000000*1000;
//                imu_msg.header.stamp.sec=imu->timestamp/1000/1000;
//                imu_msg.linear_acceleration.x=imu->accel[0];
//                imu_msg.linear_acceleration.y=imu->accel[1];
//                imu_msg.linear_acceleration.z=imu->accel[2];
//                imu_msg.angular_velocity.x=imu->gyro[0];
//                imu_msg.angular_velocity.y=imu->gyro[1];
//                imu_msg.angular_velocity.z=imu->gyro[2];
                 LOG(INFO) << "Imu count: " << imu_count;
//                 LOG(INFO) << "  frame_id: " << data.imu->frame_id
//                           << ", timestamp: " << data.imu->timestamp
//                           << ", accel_x: " << data.imu->accel[0]
//                           << ", accel_y: " << data.imu->accel[1]
//                           << ", accel_z: " << data.imu->accel[2]
//                           << ", gyro_x: " << data.imu->gyro[0]
//                           << ", gyro_y: " << data.imu->gyro[1]
//                           << ", gyro_z: " << data.imu->gyro[2]
//                           << ", temperature: " << data.imu->temperature;
            });
    api->Start(Source::ALL);
    while (true) {
        start=clock();
        api->WaitForStreams();
        end=clock();
        auto &&left_data        = api->GetStreamData(Stream::LEFT);             //获取左相机图
        auto &&right_data       = api->GetStreamData(Stream::RIGHT);            //获取右相机图
//        auto &&motion_datas     = api->GetMotionDatas();//获取IMU数据
        auto img_stamp          = left_data.img->timestamp;                     //获取时间戳
        int cnt=0;
        cv::Mat img_left_and_right,img1t,img2t;
        cv::hconcat(left_data.frame, right_data.frame, img_left_and_right);
        CvRect rect1(640,0,640,400);//图像拆分的矩形
        CvRect rect2(0,0,640,400);
        img_left_and_right(rect1).copyTo(img1t);//图像拆分
        img_left_and_right(rect2).copyTo(img2t);
        img_msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_data.frame).toImageMsg();//
        img_msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_data.frame).toImageMsg();//
        img_msg_left_and_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_left_and_right).toImageMsg();//

        img_msg_left->header.frame_id="cam";
        img_msg_left->header.stamp.nsec  =img_stamp%1000000*1000;
        img_msg_left->header.stamp.sec   =img_stamp/1000/1000;
        img_msg_right->header.frame_id="cam";
        img_msg_right->header.stamp.nsec  =img_stamp%1000000*1000;
        img_msg_right->header.stamp.sec   =img_stamp/1000/1000;

        img_msg_left_and_right->header.frame_id="cam";
        img_msg_left_and_right->header.stamp.nsec  =img_stamp%1000000*1000;
        img_msg_left_and_right->header.stamp.sec   =img_stamp/1000/1000;

        if (imu) {  //从callback读数据
            std::lock_guard<std::mutex> _(imu_mtx);
            std::cout<<"imu="<<imu->timestamp<<std::endl;
            imu_msg.header.frame_id="odom";
            imu_msg.header.stamp.nsec=imu->timestamp%1000000*1000;
            imu_msg.header.stamp.sec=imu->timestamp/1000/1000;
            imu_msg.linear_acceleration.x=imu->accel[0];
            imu_msg.linear_acceleration.y=imu->accel[1];
            imu_msg.linear_acceleration.z=imu->accel[2];
            imu_msg.angular_velocity.x=imu->gyro[0];
            imu_msg.angular_velocity.y=imu->gyro[1];
            imu_msg.angular_velocity.z=imu->gyro[2];
        }
        std::cout<<"img="<<img_stamp<<std::endl;
        if (ros::ok())//发布ROS消息
        {
            std::cout<<"ros ok"<<std::endl;
            img_pub_left.publish(img_msg_left);
            img_pub_right.publish(img_msg_right);
            img_pub_left_and_right.publish(img_msg_left_and_right);
            ros::spinOnce();
//            loop_rate.sleep();
        }
        if (ros::ok())//发布ROS消息
        {
            imu_pub.publish(imu_msg);
            ros::spinOnce();
        }
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

//        printf("time_cost=%d us,  %dms   ",end-start,(end-start)/1000);
        std::cout<<left_data.frame.size<<std::endl;
    }
    api->Stop(Source::VIDEO_STREAMING);
    return 0;
}

