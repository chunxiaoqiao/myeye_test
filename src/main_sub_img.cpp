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
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>//位姿
#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数
#include <chrono>
using  namespace cv;
using  namespace std;

Mat left_img_new,left_img_old, right_img_new,right_img_old,left_and_right_img,result,matchs_img;
char successed_flag1 =0,successed_flag2=0,successed_flag3=0,successed_flag4=0;

string topic1_name = "/mynteye/left/image_raw"; //topic 名称
string topic2_name = "/mynteye/right/image_raw";

void find_feature_matches(
        const Mat &img_1, const Mat &img_2,
        std::vector<KeyPoint> &keypoints_1,
        std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches);

void pose_estimation_2d2d(
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector<DMatch> matches,
        Mat &R, Mat &t);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

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
}
void  callback_function_right(const sensor_msgs::Image::ConstPtr  image_data)
{
    cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    pCvImage = cv_bridge::toCvShare(image_data, "bgr8");//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    right_img_new.copyTo(right_img_old);//把上一次的新的拷贝给这一次的旧的
    pCvImage->image.copyTo(right_img_new);
}

void  callback_function_left_and_right(const sensor_msgs::Image::ConstPtr  image_data)
{
    Mat temp;
    cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    pCvImage = cv_bridge::toCvShare(image_data, "bgr8");//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
    pCvImage->image.copyTo(left_and_right_img);//depth是四位图像
    successed_flag2=1;
}
void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
{
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

#pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
        const uint16_t *itI = in.ptr<uint16_t>(r);
        uint8_t *itO = tmp.ptr<uint8_t>(r);

        for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
        {
            *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
        }
    }

    cv::applyColorMap(tmp, out, COLORMAP_JET);
}

int main(int argc, char *argv[]) {
    namedWindow("left",1);
    namedWindow("left_and_right_img",1);
    namedWindow("good matches",1);
    ros::init(argc,argv,"kinect2_listen");
    if(!ros::ok())
        return 0;
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(topic1_name,50,callback_function_left);
    ros::Subscriber sub2 = n.subscribe(topic2_name,50,callback_function_right);
    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    while(ros::ok())
    {
        clock_t time_start,time_endl;
        time_start=clock();
        //-- 读取图像
        CvRect rect1(640,0,640,400);//图像拆分的矩形
        CvRect rect2(0,0,640,400);
        Mat img_1,img_2 ;
        cout << left_and_right_img.size << "  " << left_img_new.size << endl;
        if(left_img_new.empty() ||left_img_old.empty()|| right_img_old.empty()||right_img_new.empty()){
            continue;
        }
//        left_and_right_img(rect2).copyTo(img_1);//图像拆分
//        left_and_right_img(rect1).copyTo(img_2);
        img_1=left_img_old;
        img_2=left_img_new;
        assert(img_1.data && img_2.data && "Can not load images!");
        vector<KeyPoint> keypoints_1, keypoints_2;
        vector<DMatch> matches;
        find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
        imshow("left",left_img_new);
        imshow("right",right_img_new);
        imshow("good matches",matchs_img);
        waitKey(1);
        time_endl=clock();
        printf("time cost=%d\n",time_endl-time_start);
//        cout << "一共找到了" << matches.size() << "组匹配点" << endl;
        //-- 估计两张图像间运动
        Mat R, t;
        pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

        //-- 验证E=t^R*scale
        Mat t_x =
                (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
                        t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                        -t.at<double>(1, 0), t.at<double>(0, 0), 0);

//        cout << "t^R=" << endl << t_x * R << endl;

        //-- 验证对极约束
        Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
        for (DMatch m: matches) {
            Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
            Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
            Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
            Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
            Mat d = y2.t() * t_x * R * y1;
//            cout << "epipolar constraint = " << d << endl;
        }
    }
    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
    Mat img_match;
    Mat img_goodmatch;
//    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_goodmatch);
//    drawKeypoints(img_1, keypoints_1, matchs_img, Scalar(0,255,0), DrawMatchesFlags::DEFAULT);
//    imshow("all matches", img_match);
//    imshow("good matches", img_goodatch);
    matchs_img=img_goodmatch;
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
            (
                    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, Mat &t) {
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
//    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵
    Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
    double focal_length = 521;      //相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
//    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
//    cout << "homography_matrix is " << endl << homography_matrix << endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    // 此函数仅在Opencv3中提供
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;

}

