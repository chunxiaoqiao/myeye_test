//
// Created by qiaochunxiao on 2020/7/27.
//

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
#include "Stereo_Calib.h"
#include <chrono>
MYNTEYE_USE_NAMESPACE
# define M_PI		3.14159265358979323846	/* pi */
void rosmsg_make(sensor_msgs::ImagePtr img);

int main(int argc, char **argv) 
{
    std::string left("data/disp_sgbm_4/displeft");
    std::string right("data/disp_sgbm_4/dispright");
    std::string left_REC("data/disp_sgbm_4/displeft_REC");
    std::string right_REC("data/disp_sgbm_4/dispright_REC");
    std::string depth("data/disp_sgbm_4/dispdepth");
    std::string depthrec("data/disp_sgbm_4/dispdepthrec");
    std::string disp("data/disp_sgbm_4/dispdisp");
    std::string disp8("data/disp_sgbm_4/dispdisp8");
    char camcnt_ten=48;
    char camcnt_one=48;
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
    api->EnableStreamData(Stream::LEFT);
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

    cv::Mat left_img,right_img;
    left_img    =imread("data/disp_sgbm/displeft_REC00.png",IMREAD_GRAYSCALE);
    right_img   =imread("data/disp_sgbm/dispright_REC00.png",IMREAD_GRAYSCALE);
    Stereo_Calib makestereo("../../../../data/extrinsics_test_two.yml",
                            "../../../../data/intrinsics_test_two.yml",
                            1280,800,"sgbm");
//    start=clock();
//    makestereo.makedisp(left_img,right_img);
//    end=clock();
//    cout<<"time="<<end-start<<endl;
//    imshow("depth",makestereo.S_depth);
//    imshow("LEFT",left_img);
//    waitKey(0);
    namedWindow("depth", 1);
    namedWindow("left", 1);
    namedWindow("rec", 1);
    while (true) {
        start=clock();
        api->WaitForStreams();
        auto &&left_img = api->GetStreamData(Stream::LEFT);
        auto &&right_img = api->GetStreamData(Stream::RIGHT);
//
        makestereo.makedisp(left_img.frame,right_img.frame);
        end=clock();
        cout<<"time="<<end-start<<endl;

        int pointcount =rectifier.loadDepth(makestereo.S_depth);// 加载深度图，获取点云 raw_point
        rectifier.getRectifiedRGB(rectified, makestereo.img_left_r, false);// 获取配准后的RGB图
        makepoint.rgbrectifier(makestereo.img_left_r,makestereo.S_depth);//直接匹配深度图和彩色图
        makepoint.makepointclould_from_xyz(rectifier.raw_point_x,rectifier.raw_point_y,
                                           rectifier.raw_point_z,pointcount,makestereo.img_left_r);//从X，Y，Z生成点云
        imshow("depth",makestereo.S_depth);
        imshow("left",left_img.frame);
        imshow("rec",makepoint.M_rect_out);
        char q=waitKey(1);
        if (q=='q'||q=='Q')break;
    }

//    api->Stop(Source::VIDEO_STREAMING);
    return 0;
}

