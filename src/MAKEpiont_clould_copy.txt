//
// Created by qiao on 20-6-23.
//

#include "MAKEpoint_clould.h"


MAKEpoint_clould::MAKEpoint_clould() {
//    k_rgb<< 874.63529290183885223,0,616.45473674786478568,0,874.59612482800662292,400.16292108133046668,0,0,1;//1280*800
    k_rgb<<7.7835783883831414e+02, 0., 3.1108994788726733e+02, 0.,
            7.1927243307648666e+02, 2.0497628319415230e+02, 0., 0., 1.;//640*400
    pointcloud_out.resize(1280* 800);
    for(int i=0;i<1208;i++)
    {
        pointcloud_out[i].x=0;
        pointcloud_out[i].y=0;
        pointcloud_out[i].z=0;
        pointcloud_out[i].r=0;
        pointcloud_out[i].g=255;
        pointcloud_out[i].b=0;
    }
}
MAKEpoint_clould::~MAKEpoint_clould() {
   cout<<"endl"<<endl;
}
void MAKEpoint_clould::makepointclould(pcl::PointCloud<pcl::PointXYZRGB> &point_in, cv::Mat &colour) {
    int count=0;
    for(int i=0;i<point_in.size();i++)
    {
//        for(int j=0;j<IMAGE_WIDTH;j++)
//        {
//            pcl::PointXYZRGB p = point_in.at<cv::Point3f>(i, j);
            pointcloud_out[count].x=point_in[count].x;
            pointcloud_out[count].y=point_in[count].y;
            pointcloud_out[count].z=point_in[count].z;
            pointcloud_out[count].r=0;
            pointcloud_out[count].g=255;
            pointcloud_out[count].b=0;
            count++;
//        }
    }
}
void MAKEpoint_clould::makepointclould_from_xyz(float x[], float y[], float z[],int &pointcount ,cv::Mat &colour) {
    int count=0;
    int *uv_rgb;
    cv::Mat img_test(IMAGE_HEIGHT,IMAGE_WIDTH, CV_8UC3);;
    for(int i=0;i<IMAGE_HEIGHT;i++) {
        for (int j = 0; j < IMAGE_WIDTH; j++) {
            if (z[count] > 0) {
                uv_rgb=translate_rgb(x[count],y[count],z[count]);
//                cout<<*uv_rgb<<endl;
//                pcl::PointXYZRGB p = point_in.at<cv::Point3f>(i, j);
                pointcloud_out.points[count].x = x[count]/1000;
                pointcloud_out.points[count].y = y[count]/1000;
                pointcloud_out.points[count].z = z[count]/1000;
                if((*uv_rgb<IMAGE_WIDTH)&&(*(uv_rgb+1)<IMAGE_HEIGHT)&&(*uv_rgb>0)&&(*(uv_rgb+1)>0))
                {
                    pointcloud_out.points[count].r = colour.at<cv::Vec3b>(*(uv_rgb+1),*(uv_rgb))[2];
                    pointcloud_out.points[count].g = colour.at<cv::Vec3b>(*(uv_rgb+1),*(uv_rgb))[1];
                    pointcloud_out.points[count].b = colour.at<cv::Vec3b>(*(uv_rgb+1),*(uv_rgb))[0];
                }
                else
                {
                    pointcloud_out.points[count].r = 255;//colour.at<cv::Vec3b>(uv_rgb[0],uv_rgb[1])[2];
                    pointcloud_out.points[count].g = 0;//colour.at<cv::Vec3b>(uv_rgb[0],uv_rgb[1])[1];
                    pointcloud_out.points[count].b = 0;//colour.at<cv::Vec3b>(uv_rgb[0],uv_rgb[1])[0];
                }
                delete[] uv_rgb;
//                cout <<i<<"  "<<j<<"  " <<(int) colour.at<cv::Vec3b>(i, j)[1] << "  "<<(int)pointcloud_out.points[count].g<< endl;
                count++;
            }
        }
    }
//    for(int i=0;i<IMAGE_HEIGHT;i++) {
//        for (int j = 0; j < IMAGE_WIDTH; j++) {
//            img_test.at<cv::Vec3b>(i,j)[2] = (unsigned short) colour.at<cv::Vec3b>(i, j)[2];
//            img_test.at<cv::Vec3b>(i,j)[1] = (unsigned short) colour.at<cv::Vec3b>(i, j)[1];
//            img_test.at<cv::Vec3b>(i,j)[0] = (unsigned short) colour.at<cv::Vec3b>(i, j)[0];
//        }
//    }
//    imshow("img_test",img_test);
}
int *MAKEpoint_clould::translate_rgb(float &x,float &y,float &z)
{
    int *uv_out=new int[3];
    uv_out[0]=-k_rgb(0,0)*x/z+k_rgb(0,2);
    uv_out[1]=-k_rgb(1,1)*y/z+k_rgb(1,2);
    uv_out[2]=1;
//    cout<<x<<"  "<<y<<"  "<<z<<"  "<<*uv_out<<"  "<<*(uv_out+1)<<"  "<<*(uv_out+2)<<endl;
    return uv_out;
}

void MAKEpoint_clould::rgbrectifier(cv::Mat &color,cv::Mat &depth)
{
    M_rect_out= color.clone();
    for(int i=0;i<color.rows;i++)
        for(int j=0;j<color.cols;j++)
        {
            if(!depth.at<uint16_t>(i,j))
            {
                M_rect_out.at<cv::Vec3b>(i,j)[0]=0;
                M_rect_out.at<cv::Vec3b>(i,j)[1]=0;
                M_rect_out.at<cv::Vec3b>(i,j)[2]=0;
            }

        }
}