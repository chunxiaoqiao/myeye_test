//
// Created by qiaochunxiao on 2020/8/11.
//订阅双目相机，和vins位姿发布点云
//
// Created by x on 2020/4/20.
//
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <sophus/se3.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>//位姿
#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数
#include <chrono>
#include "Stereo_Calib.h"
#include "RGBRectifier.h"
#include "MAKEpoint_clould.h"
using  namespace cv;
using  namespace std;

Mat left_img_new,left_img_old, right_img_new,right_img_old,left_and_right_img,result,matchs_img;
char successed_flag1 =0,successed_flag2=0,successed_flag3=0,successed_flag4=0;

string topic1_name = "/mynteye/left/image_raw"; //topic 名称
string topic2_name = "/mynteye/right/image_raw";
string topic3_name = "/vins_estimator/keyframe_pose";
Vec3d t(0,0,0);
Eigen::Quaterniond q(1,0,0,0);
void  callback_function_left(const sensor_msgs::Image::ConstPtr  image_data)
{
    cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    //   cout<<"the frame_id:"<<image_data->frame_id.c_str()<<endl;
//    cout<<"the image heigh"<<image_data->height<<endl;
//    cout<<"the image width"<<image_data->width<<endl;
//    cout<<"the image step"<<image_data->step<<endl;
//    cout<<"listen ...."<<endl;
    pCvImage = cv_bridge::toCvShare(image_data, "bgr8");//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    left_img_new.copyTo(left_img_old);//把上一次的新的拷贝给这一次的旧的
    pCvImage->image.copyTo(left_img_new);
    successed_flag1 = 1;
    cout<<"left_seq="<<image_data->header.seq;
    printf("   left_time=%f\n",image_data->header.stamp.toSec());
}
void  callback_function_right(const sensor_msgs::Image::ConstPtr  image_data)
{
    cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    pCvImage = cv_bridge::toCvShare(image_data, "bgr8");//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    right_img_new.copyTo(right_img_old);//把上一次的新的拷贝给这一次的旧的
    pCvImage->image.copyTo(right_img_new);
    cout<<"right_seq="<<image_data->header.seq<<"   right_time="<<image_data->header.stamp.sec<<"."<<image_data->header.stamp.nsec<<endl;
}
void callback_function_odom(const nav_msgs::Odometry::ConstPtr odom)
{
    t[0] =odom->pose.pose.position.x;
    t[1] =odom->pose.pose.position.y;
    t[2] =odom->pose.pose.position.z;
    q.w() =odom->pose.pose.orientation.w;
    q.x()=odom->pose.pose.orientation.x;
    q.y()=odom->pose.pose.orientation.y;
    q.z()=odom->pose.pose.orientation.z;
    cout<<"q_seq="<<odom->header.seq<<"  ttime"<<odom->header.stamp;
    printf("   q_time=%f\n",odom->header.stamp.toSec());
    cout<<"    t="<<t<<"  q="<<q.coeffs().transpose()<<endl;
//    cout<<"x="<<x<<" y="<<y<<" z="<<z<<endl;
}
void  callback_function_left_and_right(const sensor_msgs::Image::ConstPtr  image_data)
{
    Mat temp;
    cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    pCvImage = cv_bridge::toCvShare(image_data, "bgr8");//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    pCvImage->image.copyTo(left_and_right_img);//depth是四位图像
    successed_flag2=1;
}
void clould_split(pcl::PointCloud<pcl::PointXYZRGB> &point_in,int &pointcount);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclclould(new pcl::PointCloud<pcl::PointXYZRGB>);
int main(int argc, char *argv[]) {
    namedWindow("depth", 1);
    namedWindow("left", 1);
    namedWindow("rec", 1);
    ros::init(argc,argv,"kinect2_listen");
    if(!ros::ok())
        return 0;
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(topic1_name,50,callback_function_left);
    ros::Subscriber sub2 = n.subscribe(topic2_name,50,callback_function_right);
    ros::Subscriber subodm=n.subscribe(topic3_name,50,callback_function_odom);
    ros::Publisher  pub_pcl=n.advertise<sensor_msgs::PointCloud2>("cloud_out",1);
    sensor_msgs::PointCloud2 out_put;
    ros::AsyncSpinner spinner(3); // Use 3 threads

    Matrix3f k_depth_cam;//配准相关代码  640*400
    k_depth_cam << 7.7835783883831414e+02, 0., 3.1108994788726733e+02, 0.,
            7.1927243307648666e+02, 2.0497628319415230e+02, 0., 0., 1.;
    Matrix3f k_rgb_cam;
    k_rgb_cam << 7.7835783883831414e+02, 0., 3.1108994788726733e+02, 0.,
            7.1927243307648666e+02, 2.0497628319415230e+02, 0., 0., 1.;
    Matrix3f r_depth_rgb;
    r_depth_rgb << 1,0,0,0,1,0,0,0,1;
    RGBRectifier rectifier(k_depth_cam,k_rgb_cam,r_depth_rgb,0,0,0);//80.51362369119924267, -0.03951003994504062, 0.88129144764433631);
    cv::Mat  rectified;
    MAKEpoint_clould makepoint;
    Stereo_Calib makestereo("../../../../data/extrinsics_640_400.yml",
                            "../../../../data/intrinsics_640_400.yml",
                            640,400,"sgbm");
    spinner.start();
    while(ros::ok())
    {
        clock_t time_start,time_endl;
        time_start=clock();
        //-- 读取图像
        CvRect rect1(640,0,640,400);//图像拆分的矩形
        CvRect rect2(0,0,640,400);
        Mat img_1,img_2 ;
//        cout << left_and_right_img.size << "  " << left_img_new.size << endl;
        if(left_img_new.empty() ||left_img_old.empty()|| right_img_old.empty()||right_img_new.empty()){
            continue;
        }
//        left_and_right_img(rect2).copyTo(img_1);//图像拆分
//        left_and_right_img(rect1).copyTo(img_2);
        printf("+++++++++++++++++++++++\n");
        img_1=left_img_new;
        img_2=right_img_new;
        makestereo.makedisp(img_1,img_2);
        int pointcount =rectifier.loadDepth(makestereo.S_depth);// 加载深度图，获取点云 raw_point
        rectifier.getRectifiedRGB(rectified, makestereo.img_left_r, false);// 获取配准后的RGB图
        makepoint.rgbrectifier(makestereo.img_left_r,makestereo.S_depth);//直接匹配深度图和彩色图
        makepoint.makepointclould_from_xyz(rectifier.raw_point_x,rectifier.raw_point_y,
                                           rectifier.raw_point_z,pointcount,makestereo.img_left_r);//从X，Y，Z生成点云
        pclclould->clear();
        clould_split(makepoint.pointcloud_out,pointcount);
        pcl::toROSMsg(*pclclould,out_put);
        out_put.header.frame_id="world";
        imshow("depth",makestereo.S_depth);
        imshow("left",img_1);
        imshow("rec",makepoint.M_rect_out);
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            pclclould->is_dense=false;
            pcl::io::savePCDFileBinary("../../../../data_img_depth/map1.pcd",*pclclould);
            printf("success");
            break;
        }
        if(ros::ok())
        {
            pub_pcl.publish(out_put);
            ros::spinOnce();
        }
        time_endl=clock();
//        printf("time cost=%d\n",time_endl-time_start);
    }
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}

void clould_split(pcl::PointCloud<pcl::PointXYZRGB> &point_in,int &pointcount)
{
//    Sophus::SE3 post(q,
//            Eigen::Vector3d(t[0],t[1],t[2]));
    Eigen::Matrix3d R=q.toRotationMatrix();
    for(int i=0;i<pointcount;i++)
    { 
        Eigen::Vector3d point;
        point[0]=point_in.points[i].x;
        point[1]=point_in.points[i].y;
        point[2]=point_in.points[i].z;
        Eigen::Vector3d pointWorld = R*point;//post * point;
        pcl::PointXYZRGB p;
        p.x=pointWorld[0]+t[0];
        p.y=pointWorld[1]+t[1];
        p.z=pointWorld[2]+t[2];
        p.r=point_in.points[i].r;
        p.g=point_in.points[i].g;
        p.b=point_in.points[i].b;
        pclclould->push_back(p);
    }
    printf("--------------------------------\n");

}

