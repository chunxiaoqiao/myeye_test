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
#include "RGBRectifier.h"
#include "MAKEpoint_clould.h"
#include <chrono>
MYNTEYE_USE_NAMESPACE
int main(int argc, char *argv[]) {
    clock_t start,end;
    ros::init (argc, argv, "public_ros");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image> ("img_out", 1);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::PointCloud2 output;
    sensor_msgs::ImagePtr img_msg;
    // Fill in the cloud data
    cloud.width  = 1280;
    cloud.height = 800;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
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
    api->SetOptionValue(Option::IR_CONTROL, 80);
    api->SetDisparityComputingMethodType(DisparityComputingMethod::BM);
    api->EnableStreamData(Stream::DISPARITY_NORMALIZED);
    api->EnableStreamData(Stream::POINTS);
    api->EnableStreamData(Stream::DEPTH);
    api->Start(Source::VIDEO_STREAMING);
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
    while (true) {
        start=clock();
        api->WaitForStreams();
        auto &&left_data        = api->GetStreamData(Stream::LEFT);             //获取左相机图
        auto &&right_data       = api->GetStreamData(Stream::RIGHT);            //获取右相机图
        auto &&right_data_REC   = api->GetStreamData(Stream::LEFT_RECTIFIED);   //获取畸变校正后图
//        auto &&points_data      = api->GetStreamData(Stream::POINTS);           //获取点云
//        auto &&disp_data        = api->GetStreamData(Stream::DISPARITY_NORMALIZED);//获取深度可显示数据
        auto &&depth_data       = api->GetStreamData(Stream::DEPTH);               //获取深度原始数据
        auto img_stamp          = left_data.img->timestamp;                     //获取时间戳
//        auto img_stamp2         = right_data.img->timestamp;                    //获取时间戳
        int cnt=0;
//        while(right_data_REC.frame.empty())
//        {
//            api->WaitForStreams();
//            cnt++;
//            right_data_REC   = api->GetStreamData(Stream::LEFT_RECTIFIED);   //获取畸变校正后图
//        }printf("cnt= %d;  ",cnt);
        if(right_data_REC.frame.empty())
            continue;
        printf("time=%d us  %d ms  %d s %d\n",(int)img_stamp,(int64)img_stamp/1000,(int64)img_stamp/1000/1000,0);//
//        cout<<"stamp="<<img_stamp<<"   img_stamp2="<<img_stamp2<<endl;
//        cv::Mat img;
//        cv::hconcat(left_data.frame, right_data.frame, img);//图像拼接
//        painter.DrawImgData(img, *left_data.img);//图片打上文字
//        cv::imshow("frame", img);
        // LOG(INFO) << "left id: " << left_data.frame_id
        //     << ", right id: " << right_data.frame_id;
        int pointcount =rectifier.loadDepth(depth_data.frame);// 加载深度图，获取点云 raw_point
        rectifier.getRectifiedRGB(rectified, right_data_REC.frame, false);// 获取配准后的RGB图
//        if(!rectified.empty())imshow("right_rectified",rectified);

//        makepoint.makepointclould_from_xyz(rectifier.raw_point_x,rectifier.raw_point_y,
//                                                rectifier.raw_point_z,pointcount,right_data_REC.frame);//从X，Y，Z生成点云

//        pcl::toROSMsg(makepoint.pointcloud_out, output);
//        output.header.frame_id = "odom";
//        ros::Time current_time = ros::Time::now();
//        output.header.stamp=current_time;
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_data_REC.frame).toImageMsg();//mono16
        if (ros::ok())//发布ROS消息
        {

            pcl_pub.publish(output);
            img_pub.publish(img_msg);
            ros::spinOnce();
//            loop_rate.sleep();

        }

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            makepoint.pointcloud_out.is_dense=false;
            pcl::io::savePCDFileBinary("map_qiao_REC_LEFT_xyz_zhudongguang_no.pcd",makepoint.pointcloud_out);
//        pcl::io::savePCDFileBinary("map_qiao_REC_LEFT_cloud_3.pcd",cloud);
            break;
        }
//        if (pcviewer.WasStopped()) {
//            break;
//        }
        end=clock();
        printf("time_cost=%d us,  %dms   ",end-start,(end-start)/1000);
        cout<<left_data.frame.size<<endl;
    }

    api->Stop(Source::VIDEO_STREAMING);
    return 0;
}

