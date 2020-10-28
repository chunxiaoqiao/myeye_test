//
// Created by qiaochunxiao on 2020/7/27.
//

#ifndef MYEYE_TEST_STERRO_CALIB_H
#define MYEYE_TEST_STERRO_CALIB_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <cmath>
#include <chrono>
using namespace cv;
using namespace std;
class Stereo_Calib {
private:
    int S_max_disparity=80;//双目深度的参数
    int S_blocksize=7;//双目深度的参数
    bool S_no_display=0;//双目深度的参数
    int S_help=0;//双目深度的参数
    bool S_scale=1;//双目深度的参数
    int alg;
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    Size img_size;
    Rect roi1, roi2;
    Mat Q;
    Mat M1, D1, M2, D2;
    Mat R, T, R1, P1, R2, P2;
    Mat map11, map12, map21, map22;
    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    std::string img1_filename = "";
    std::string img2_filename = "";
    std::string intrinsic_filename = "";
    std::string extrinsic_filename = "";
    std::string disparity_filename = "";
    std::string point_cloud_filename = "pointclould_test";
public:
    std::string S_extrinsics;//外参文件名
    std::string S_intrinsics;//内参文件名
    int S_img_width;//图片尺寸
    int S_img_height;//
    std::string S_algorithm;//sgbm或bm
    cv::Mat img_left_r,img_right_r;
    cv::Mat S_depth;//深度图
    Stereo_Calib(std::string extrinsics,std::string intrinsics,
                 int img_width,int img_height,
                 std::string algorithm);//构造函数，列表初始化
    ~Stereo_Calib();
    int init();
    int makedisp(const cv::Mat &left_img_input,    const cv::Mat &right_img_input);
    void print_help();
    void saveXYZ(const char* filename, const Mat& mat);
};


#endif //MYEYE_TEST_STERRO_CALIB_H
