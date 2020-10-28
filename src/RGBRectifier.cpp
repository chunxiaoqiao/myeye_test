//
// Created by x on 2020/4/10.
//

#include "RGBRectifier.h"

//left equidistant, width: 1280, height: 800, k2: -0.14643717897419709, k3: 0.01487662528323725, k4: -0.12909823085068414, k5: 0.12520483515087141, mu: 879.58586571849730262, mv: 879.23603278744235467, u0: 623.75239698472819327, v0: 402.99155169680261679
//right equidistant, width: 1280, height: 800, k2: -0.15526500755177633, k3: 0.02378814714263805, k4: -0.14317865104131694, k5: 0.19998878882775445, mu: 877.79277417962578056, mv: 877.27556420931466619, u0: 620.45264947944372125, v0: 393.76784158522150392
//depth fps:

RGBRectifier::RGBRectifier(Matrix3f k_depth_cam,Matrix3f k_rgb_cam , Matrix3f rotation,
                           double tx, double ty, double tz)
{

    K_Depth_Cam = k_depth_cam;
    K_Rgb_Cam = k_rgb_cam;

    Depth_Rgb_Rotation = rotation;
    Depth_Rgb_Translation[0] = tx;
    Depth_Rgb_Translation[1] = ty;
    Depth_Rgb_Translation[2] = tz;

    raw_point_x = new float[IMAGE_WIDTH*IMAGE_HEIGHT];
    raw_point_y = new float[IMAGE_WIDTH*IMAGE_HEIGHT];
    raw_point_z = new float[IMAGE_WIDTH*IMAGE_HEIGHT];

    trans_point_x = new float[IMAGE_WIDTH*IMAGE_HEIGHT];
    trans_point_y = new float[IMAGE_WIDTH*IMAGE_HEIGHT];
    trans_point_z = new float[IMAGE_WIDTH*IMAGE_HEIGHT];

    pixel_x = new int[IMAGE_WIDTH*IMAGE_HEIGHT];
    pixel_y = new int[IMAGE_WIDTH*IMAGE_HEIGHT];

    pixel_r = new unsigned char[IMAGE_WIDTH*IMAGE_HEIGHT];
    pixel_g = new unsigned char[IMAGE_WIDTH*IMAGE_HEIGHT];
    pixel_b = new unsigned char[IMAGE_WIDTH*IMAGE_HEIGHT];

}

RGBRectifier::~RGBRectifier()
{
    delete pixel_x;
    delete pixel_y;

    delete pixel_r;
    delete pixel_g;
    delete pixel_b;

    delete trans_point_x;
    delete trans_point_y;
    delete trans_point_z;

    delete raw_point_x;
    delete raw_point_y;
    delete raw_point_z;
}


int RGBRectifier::loadPoints(Mat &depth)
{
    int count = 0;
    Vector3f p;
    Vector2i pixelp;

    int x;
    int y;

    unsigned short * depth_array = (unsigned short *)depth.ptr();/////数据类型改变
    float d;

    float kdc_00 = K_Depth_Cam(0,0);
    float kdc_02 = K_Depth_Cam(0,2);
    float kdc_11 = K_Depth_Cam(1,1);
    float kdc_12 = K_Depth_Cam(1,2);

    float kdc_00_i  =1.0/kdc_00;
    float kdc_11_i  =1.0/kdc_11;

    int index = 0;

    for(int i=0;i<depth.rows;i++)
        for(int j=0;j<depth.cols;j++)
        {
            d  = depth_array[index];
            if(d>0)
            {
                raw_point_z[count] = d;//生成点云
                raw_point_x[count] = (kdc_02-j)*d*kdc_00_i;
                raw_point_y[count] = (kdc_12-i)*d*kdc_11_i;

                pixel_x[count] = j;
                pixel_y[count] = i;
                count ++;
            }
            index++;
        }

    return  count;
}

void RGBRectifier::translatePoints()
{

    float r00 = Depth_Rgb_Rotation(0,0);
    float r01 = Depth_Rgb_Rotation(0,1);
    float r02 = Depth_Rgb_Rotation(0,2);
    float r10 = Depth_Rgb_Rotation(1,0);
    float r11 = Depth_Rgb_Rotation(1,1);
    float r12 = Depth_Rgb_Rotation(1,2);
    float r20 = Depth_Rgb_Rotation(2,0);
    float r21 = Depth_Rgb_Rotation(2,1);
    float r22 = Depth_Rgb_Rotation(2,2);

    float px,py,pz;
    float tx,ty,tz;

    tx = Depth_Rgb_Translation[0];
    ty = Depth_Rgb_Translation[1];
    tz = Depth_Rgb_Translation[2];

    for(int i=0;i<point_number;i++)
    {
        px = raw_point_x[i];
        py = raw_point_y[i];
        pz = raw_point_z[i];

        trans_point_x[i] = r00 * px + r01 * py + r02 * pz + tx;
        trans_point_y[i] = r10 * px + r11 * py + r12 * pz + ty;
        trans_point_z[i] = r20 * px + r21 * py + r22 * pz + tz;
    }
}

int RGBRectifier::loadDepth(Mat &depth)
{
    int points;
    points = loadPoints(depth);
    point_number = points;
    translatePoints();

    return points;
}

void RGBRectifier::getRectifiedRGB(Mat &rectifiedRGB,Mat &color,bool toFillHole)
{

    if(toFillHole)
        color.copyTo(rectifiedRGB);
    else
        rectifiedRGB = Mat::zeros(color.rows,color.cols,CV_8UC3);

    float z_i;
    int src_pix_x,src_pix_y;

    float krc_00 = K_Rgb_Cam(0,0);
    float krc_02 = K_Rgb_Cam(0,2);
    float krc_11 = K_Rgb_Cam(1,1);
    float krc_12 = K_Rgb_Cam(1,2);
    unsigned  char * rgb_ptr = rectifiedRGB.data;
    unsigned  char * src_ptr = color.data;
    int step = rectifiedRGB.cols;

    int index_dst;
    int index_src;
    for(int i=0;i<point_number;i++)
    {
        z_i = 1.0/trans_point_z[i];
        src_pix_x = -trans_point_x[i]*krc_00*z_i+krc_02;
        src_pix_y = -trans_point_y[i]*krc_11*z_i+krc_12;

        if(src_pix_x<0)
            continue;
        if(src_pix_x>=color.cols)
            continue;

        if(src_pix_y<0)
            continue;
        if(src_pix_y>=color.rows)
            continue;

        index_dst = (pixel_y[i]*rectifiedRGB.cols+pixel_x[i])*3;
        index_src = (src_pix_y*color.cols+src_pix_x)*3;
        rgb_ptr[index_dst] = src_ptr[index_src];
        rgb_ptr[index_dst+1] = src_ptr[index_src+1];
        rgb_ptr[index_dst+2] = src_ptr[index_src+2];

        //rectifiedRGB.at<Vec3b>(pixel_y[i],pixel_x[i]) =  color.at<Vec3b>(src_pix_y,src_pix_x);
    }
    //return  rectifiedRGB;
}

void RGBRectifier::getRectified2DPoints(Mat &color)
{
    float z_i;
    int src_pix_x,src_pix_y;

    float krc_00 = K_Rgb_Cam(0,0);
    float krc_02 = K_Rgb_Cam(0,2);
    float krc_11 = K_Rgb_Cam(1,1);
    float krc_12 = K_Rgb_Cam(1,2);


    for(int i=0;i<point_number;i++)
    {
        z_i = 1.0/trans_point_z[i];
        src_pix_x = -trans_point_x[i]*krc_00*z_i+krc_02;//配准
        src_pix_y = -trans_point_y[i]*krc_11*z_i+krc_12;

        //dst_pos = PointsPixel[i];

        if(src_pix_x<0)
            continue;
        if(src_pix_x>=color.cols)
            continue;

        if(src_pix_y<0)
            continue;
        if(src_pix_y>=color.rows)
            continue;

        pixel_b[i] = color.at<Vec3b>(src_pix_y,src_pix_x)[0];
        pixel_g[i] = color.at<Vec3b>(src_pix_y,src_pix_x)[1];
        pixel_r[i] = color.at<Vec3b>(src_pix_y,src_pix_x)[2];

    }
}

void RGBRectifier::getRectified3DPoints(Mat &color)
{
    float z_i;
    int src_pix_x,src_pix_y;

    float krc_00 = K_Rgb_Cam(0,0);
    float krc_02 = K_Rgb_Cam(0,2);
    float krc_11 = K_Rgb_Cam(1,1);
    float krc_12 = K_Rgb_Cam(1,2);

    unsigned char *color_data = color.data;
    unsigned char *bgr;

    for(int i=0;i<point_number;i++)
    {
        z_i = 1.0/trans_point_z[i];
        src_pix_x = -trans_point_x[i]*krc_00*z_i+krc_02;
        src_pix_y = -trans_point_y[i]*krc_11*z_i+krc_12;
        if(src_pix_x<0)
            continue;
        if(src_pix_x>=color.cols)
            continue;

        if(src_pix_y<0)
            continue;
        if(src_pix_y>=color.rows)
            continue;
        bgr = (unsigned char *)(color_data + (src_pix_y * color.cols + src_pix_x)*3 );
        pixel_b[i] = bgr[0];
        pixel_g[i] = bgr[1];
        pixel_r[i] = bgr[2];
    }
}

