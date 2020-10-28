//
// Created by x on 2020/4/10.
//
//   算法改动 明显提高运算效率
//   1.以std::vector存放三维点和二维点的效率太低，改为定长数组的形式，长度暂定为1280×800个点
//   2.避免频繁使用固定的计算式，尽量把计算放在for之前，取得与for无关的中间值
//   3.避免使用除法，可改为乘以倒数的形式
//   4.矩阵向量运算时尽量避免使用Eigen的运算

#ifndef UNTITLED_RGBRECTIFIER_H
#define UNTITLED_RGBRECTIFIER_H
#include<iostream>
#include "Eigen/Eigen"
#include "opencv2/opencv.hpp"
#include "vector"
using  namespace Eigen;
using  namespace cv;
using  namespace std;

class RGBRectifier {

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 800


private:
    Matrix3f Depth_Rgb_Rotation;
    Vector3f Depth_Rgb_Translation;
    Matrix3f K_Depth_Cam;
    Matrix3f K_Rgb_Cam;

public:

    float *raw_point_x;
    float *raw_point_y;
    float *raw_point_z;

    float *trans_point_x;
    float *trans_point_y;
    float *trans_point_z;

    int *pixel_x;
    int *pixel_y;

    unsigned char *pixel_r;
    unsigned char *pixel_g;
    unsigned char *pixel_b;

    int point_number = 0;

    RGBRectifier(Matrix3f k_depth_cam,Matrix3f k_rgb_cam , Matrix3f rotation,
                 double tx, double ty, double tz);

    ~RGBRectifier();

    int loadPoints(Mat &depth);

    void translatePoints();

    int loadDepth(Mat &depth);

    void getRectifiedRGB(Mat &rectifiedRGB,Mat &color,bool toFillHole);

    void getRectified2DPoints(Mat &color);
    void getRectified3DPoints(Mat &color);

};


#endif //UNTITLED_RGBRECTIFIER_H
