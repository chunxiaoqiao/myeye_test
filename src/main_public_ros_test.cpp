//
// Created by qiao on 20-7-3.
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
#include "mynteye/api/api.h"
#include "util_cv.h"
#include "util_pcl.h"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "RGBRectifier.h"
#include "MAKEpoint_clould.h"
#include <chrono>
MYNTEYE_USE_NAMESPACE
# define M_PI		3.14159265358979323846	/* pi */
void rosmsg_make(sensor_msgs::ImagePtr img);

int main(int argc, char *argv[]) {
    std::string left("data/disp_sgbm_640/displeft");
    std::string right("data/disp_sgbm_640/dispright");

    std::string left_REC("data/disp_sgbm_5/displeft_REC");
    std::string right_REC("data/disp_sgbm_5/dispright_REC");

    std::string depth("data/disp_sgbm_640/dispdepth");
    std::string depthrec("data/disp_sgbm_5/dispdepthrec");

    std::string disp("data/disp_sgbm_640/dispdisp");
    std::string disp8("data/disp_sgbm_640/dispdisp8");
    char camcnt_ten=48;
    char camcnt_one=52;
    clock_t start,end;
    ros::init (argc, argv, "public_ros");
    ros::NodeHandle nh;

    ros::Publisher pcl_pub      = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Publisher img_pub      = nh.advertise<sensor_msgs::Image> ("/cam0/image_raw", 1);
    ros::Publisher depth_pub    = nh.advertise<sensor_msgs::Image> ("depth_out", 1);
    ros::Publisher imu_pub      = nh.advertise<sensor_msgs::Imu> ("imu0", 20);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;//pcl点云
    sensor_msgs::PointCloud2 output;//点云
    sensor_msgs::ImagePtr   img_msg;//rgb图
    sensor_msgs::ImagePtr   depth_msg;//深度图
    sensor_msgs::Imu     imu_msg;


    cloud.width  = 1280;
    cloud.height = 800;
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t i = 0; i < cloud.points.size (); ++i)// 填充点云 初始化
    {
        cloud.points[i].x =0;// 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y =0;// 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z =0; //1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].r = 0;
        cloud.points[i].g = 0;
        cloud.points[i].b = 0;
    }
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";
    ros::Rate loop_rate(1);

    auto &&api = API::Create(argc, argv);
    if (!api) return 1;
    bool ok;
    auto &&request = api->SelectStreamRequest(&ok);
    if (!ok) return 1;

    api->ConfigStreamRequest(request);
    api->EnableStreamData(Stream::DEPTH);
    api->EnableStreamData(Stream::LEFT_RECTIFIED);
    api->SetDisparityComputingMethodType(DisparityComputingMethod::SGBM);
    api->EnableStreamData(Stream::DISPARITY_NORMALIZED);
    api->EnableMotionDatas();
    api->Start(Source::ALL);

    CVPainter painter;
    PCViewer pcviewer;

    Matrix3f k_depth_cam;//配准相关代码
    k_depth_cam << 879.58586,0,623.75239,0,879.23603,402.99155,0,0,1;
    Matrix3f k_rgb_cam;
    k_rgb_cam << 877.792774,0,620.452649,0,877.275564,393.76784,0,0,1;
    Matrix3f r_depth_rgb;
    r_depth_rgb << 1,0,0,0,1,0,0,0,1;
    RGBRectifier rectifier(k_depth_cam,k_rgb_cam,r_depth_rgb,0,0,0);//80.51362369119924267, -0.03951003994504062, 0.88129144764433631);
    cv::Mat  rectified;
    MAKEpoint_clould makepoint;
    double gravity_=9.8;
    while (true) {
        start=clock();
        api->WaitForStreams();
        auto &&left_data        = api->GetStreamData(Stream::LEFT);             //获取左相机图
        auto &&right_data       = api->GetStreamData(Stream::RIGHT);            //获取右相机图

        auto &&left_data_REC    = api->GetStreamData(Stream::LEFT_RECTIFIED);   //获取畸变校正后图
        auto &&right_data_REC   = api->GetStreamData(Stream::RIGHT_RECTIFIED);   //获取畸变校正后图
        auto &&depth_data       = api->GetStreamData(Stream::DEPTH);               //获取深度原始数据
        auto img_stamp          = left_data.img->timestamp;                     //获取时间戳
        auto &&motion_datas = api->GetMotionDatas();//获取IMU数据
        auto &&disp_norm_data = api->GetStreamData(Stream::DISPARITY_NORMALIZED);//获取视差图(显示用)
        auto &&disp_data = api->GetStreamData(Stream::DISPARITY);//获取视差图
        int cnt=0;
        while(left_data_REC.frame.empty())
        {
            api->WaitForStreams();
            cnt++;
            left_data_REC   = api->GetStreamData(Stream::LEFT_RECTIFIED);   //获取畸变校正后图
        }printf("cnt= %d;  ",cnt);cnt=0;
        while(motion_datas.empty())
        {
            cnt++;
            motion_datas = api->GetMotionDatas();//获取IMU数据
            printf("!!cnt= %d;  ",cnt);
        }
        imshow("left",left_data.frame);
        if (!disp_norm_data.frame.empty()) {//显示视差图
            cv::imshow("disparity_normalized", disp_norm_data.frame);  // CV_8UC1
        }
        if (!disp_data.frame.empty()) {//显示视差图
            cv::imshow("disparity", disp_data.frame);  // CV_8UC1
        }
//        for(int i=0;(i<1280*400)&&!disp_data.frame.empty()&&!disp_norm_data.frame.empty();i=i+10)
//        {
//            cout<<"disp8="<<(int)disp_norm_data.frame.data[i]<<"  disp="<<(int)disp_data.frame.data[i]<<endl;
//        }
//        printf("\ttime=%d us\t%d ms  %d s %d    ",(int)img_stamp,(int64)img_stamp/1000,(int64)img_stamp/1000/1000,0);//
        int pointcount =rectifier.loadDepth(depth_data.frame);// 加载深度图，获取点云 raw_point
        rectifier.getRectifiedRGB(rectified, left_data_REC.frame, false);// 获取配准后的RGB图
        if(!rectified.empty())imshow("right_rectified",rectified);
        makepoint.makepointclould_from_xyz(rectifier.raw_point_x,rectifier.raw_point_y,
                                                rectifier.raw_point_z,pointcount,left_data_REC.frame);//从X，Y，Z生成点云

        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_data_REC.frame).toImageMsg();//彩色图
        img_msg->header.frame_id="odom";
        img_msg->header.stamp.nsec  =img_stamp%1000000*1000;
        img_msg->header.stamp.sec   =img_stamp/1000/1000;

        depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_data.frame).toImageMsg();//深度图
        depth_msg->header.frame_id="odom";
        depth_msg->header.stamp.nsec  =img_stamp%1000000*1000;
        depth_msg->header.stamp.sec   =img_stamp/1000/1000;

        int ic=0;
        for (auto &&data : motion_datas)
        {
            ic++;
            if(img_stamp<data.imu->timestamp&&ic<motion_datas.size()||ic==(motion_datas.size()-1))
            {
                imu_msg.header.frame_id="odom";
                imu_msg.header.stamp.nsec=data.imu->timestamp%1000000*1000;
                imu_msg.header.stamp.sec=data.imu->timestamp/1000/1000;
                imu_msg.linear_acceleration.x=(data.imu->accel[0]+motion_datas[ic].imu->accel[0])*gravity_;
                imu_msg.linear_acceleration.y=(data.imu->accel[1]+motion_datas[ic].imu->accel[1])*gravity_;
                imu_msg.linear_acceleration.z=(data.imu->accel[2]+motion_datas[ic].imu->accel[2])*gravity_;
                imu_msg.angular_velocity.x=(data.imu->gyro[0]+motion_datas[ic].imu->gyro[0])* M_PI / 180;
                imu_msg.angular_velocity.y=(data.imu->gyro[1]+motion_datas[ic].imu->gyro[1])* M_PI / 180;
                imu_msg.angular_velocity.z=(data.imu->gyro[2]+motion_datas[ic].imu->gyro[2])* M_PI / 180;
                break;
            }
        }
//        if(!motion_datas.empty())
//        {
//            imu_msg.header.frame_id="odom";
//            imu_msg.header.stamp.nsec=motion_datas[0].imu->timestamp%1000000*1000;
//            imu_msg.header.stamp.sec=motion_datas[0].imu->timestamp/1000/1000;
//            imu_msg.angular_velocity.x=motion_datas[0].imu->accel[0];
//            imu_msg.angular_velocity.y=motion_datas[0].imu->accel[1];
//            imu_msg.angular_velocity.z=motion_datas[0].imu->accel[2];
//            imu_msg.linear_acceleration.x=motion_datas[1].imu->gyro[0];
//            imu_msg.linear_acceleration.y=motion_datas[1].imu->gyro[1];
//            imu_msg.linear_acceleration.z=motion_datas[1].imu->gyro[2];
//        }


        if (ros::ok())//发布ROS消息
        {
            pcl_pub.publish(output);
            img_pub.publish(img_msg);
            depth_pub.publish(depth_msg);
            imu_pub.publish(imu_msg);
            ros::spinOnce();
        }

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            makepoint.pointcloud_out.is_dense=false;
            pcl::io::savePCDFileBinary("map_lab.pcd",makepoint.pointcloud_out);
            imwrite("depth.png",depth_data.frame);
            imwrite("leftREC.png",left_data_REC.frame);
            imwrite("rightREC.png",right_data_REC.frame);
            imwrite("left.png",left_data.frame);
            imwrite("right.png",right_data.frame);
            imwrite("dep_RGB.png",rectified);

            break;
        }
        if ((key == 26 || key == 'p' || key == 'P')
                &&  !disp_data.frame.empty()      &&    !disp_norm_data.frame.empty()
//                &&  !left_data_REC.frame.empty()  &&    !right_data_REC.frame.empty()
//                &&  !depth_data.frame.empty()     &&    !rectified.empty())
                )
                {  // 存储
//            imwrite(left+camcnt_ten+camcnt_one+".png",left_data_REC.frame);
//            imwrite(right+camcnt_ten+camcnt_one+".png",right_data_REC.frame);

            imwrite(left+camcnt_ten+camcnt_one+".png",left_data.frame);
            imwrite(right+camcnt_ten+camcnt_one+".png",right_data.frame);

            imwrite(depth+camcnt_ten+camcnt_one+".png",depth_data.frame);
//            imwrite(depthrec+camcnt_ten+camcnt_one+".png",rectified);

            imwrite(disp+camcnt_ten+camcnt_one+".png",disp_data.frame);
            imwrite(disp8+camcnt_ten+camcnt_one+".png",disp_norm_data.frame);
            if(camcnt_one<57)camcnt_one++;
            else{camcnt_one=48;camcnt_ten++;}
        }
        end=clock();
        printf("time_cost=%d us,  %dms  \n ",end-start,(end-start)/1000);
    }

    api->Stop(Source::VIDEO_STREAMING);
    return 0;
}
void rosmsg_make(sensor_msgs::ImagePtr img)
{
    ;
}

