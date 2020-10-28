//
// Created by qiao on 20-6-30.
//

#ifndef MYEYE_TEST_FEATURE_H
#define MYEYE_TEST_FEATURE_H
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
class Feature {
private:
public:
    Feature();
    ~Feature();
    // 像素坐标转相机归一化坐标
    Point2d pixel2cam ( const Point2d& p, const Mat& K );
    void feature_find(Mat& img1,Mat& img2);//这个类是自己写的
    void find_feature_matches (
            const Mat& img_1, const Mat& img_2,//const 其中Mat&  是参数类型，&表示引用类型，img1是参数
            std::vector<KeyPoint>& keypoints_1,//标准库类型，类模板定义。keypoint2是keypoint类型，应该也是引用，节省内存空间
            std::vector<KeyPoint>& keypoints_2,
            std::vector< DMatch >& matches );

    void pose_estimation_2d2d (
            std::vector<KeyPoint> keypoints_1,
            std::vector<KeyPoint> keypoints_2,
            std::vector< DMatch > matches,
            Mat& R, Mat& t );
};


#endif //MYEYE_TEST_FEATURE_H
