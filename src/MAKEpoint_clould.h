//
// Created by qiao on 20-6-23.
//

#ifndef MYEYE_TEST_MAKEPOINT_CLOULD_H
#define MYEYE_TEST_MAKEPOINT_CLOULD_H
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Eigen/Eigen"
#include "opencv2/opencv.hpp"
#include "vector"
using  namespace Eigen;
using  namespace cv;
using  namespace std;

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 800

class MAKEpoint_clould {
private:
    Eigen::Matrix3f k_rgb;
public:
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_out;
    cv::Mat M_rect_out;
    MAKEpoint_clould();
    ~MAKEpoint_clould();
    void makepointclould(pcl::PointCloud<pcl::PointXYZRGB> &point_in,cv::Mat &colour);
    void makepointclould_from_xyz(float x[], float y[], float z[],int &pointcount ,cv::Mat &colour) ;
    int *translate_rgb(float &x,float &y,float &z);
    void rgbrectifier(cv::Mat &color,cv::Mat &depth);
};


#endif //MYEYE_TEST_MAKEPOINT_CLOULD_H
