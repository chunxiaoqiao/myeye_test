//
// Created by qiao on 20-6-30.
//
// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include "mynteye/logger.h"
#include "mynteye/api/api.h"

#include "util_cv.h"
#include "util_pcl.h"

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "RGBRectifier.h"
#include "MAKEpoint_clould.h"
#include "Feature.h"

namespace {

    class DepthRegion {
    public:
        explicit DepthRegion(std::uint32_t n)
                : n_(std::move(n)), show_(false), selected_(false), point_(0, 0) {}

        ~DepthRegion() = default;

        /**
         * 鼠标事件：默认不选中区域，随鼠标移动而显示。单击后，则会选中区域来显示。你可以再单击已选中区域或双击未选中区域，取消选中。
         */
        void OnMouse(const int &event, const int &x, const int &y, const int &flags) {
            MYNTEYE_UNUSED(flags)//没有用处的标记
            if (event != cv::EVENT_MOUSEMOVE && event != cv::EVENT_LBUTTONDOWN) {
                return;
            }
            show_ = true;

            if (event == cv::EVENT_MOUSEMOVE) {
                if (!selected_) {
                    point_.x = x;
                    point_.y = y;
                }
            } else if (event == cv::EVENT_LBUTTONDOWN) {
                if (selected_) {
                    if (x >= static_cast<int>(point_.x - n_) &&
                        x <= static_cast<int>(point_.x + n_) &&
                        y >= static_cast<int>(point_.y - n_) &&
                        y <= static_cast<int>(point_.y + n_)) {
                        selected_ = false;
                    }
                } else {
                    selected_ = true;
                }
                point_.x = x;
                point_.y = y;
            }
        }

        template <typename T>     //类模板
        void ShowElems(
                const cv::Mat &depth,
                std::function<std::string(const T &elem)> elem2string,
                int elem_space = 40,
                std::function<std::string(
                        const cv::Mat &depth, const cv::Point &point, const std::uint32_t &n)>
                getinfo = nullptr) {
            if (!show_)
                return;

            int space = std::move(elem_space);
            int n = 2 * n_ + 1;
            cv::Mat im(space * n, space * n, CV_8UC3, cv::Scalar(255, 255, 255));

            int x, y;
            std::string str;
            int baseline = 0;
            for (int i = -n_; i <= n; ++i) {
                x = point_.x + i;
                if (x < 0 || x >= depth.cols)
                    continue;
                for (int j = -n_; j <= n; ++j) {
                    y = point_.y + j;
                    if (y < 0 || y >= depth.rows)
                        continue;

                    str = elem2string(depth.at<T>(y, x));

                    cv::Scalar color(0, 0, 0);
                    if (i == 0 && j == 0)
                        color = cv::Scalar(0, 0, 255);

                    cv::Size sz =
                            cv::getTextSize(str, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

                    cv::putText(
                            im, str, cv::Point(
                                    (i + n_) * space + (space - sz.width) / 2,
                                    (j + n_) * space + (space + sz.height) / 2),
                            cv::FONT_HERSHEY_PLAIN, 1, color, 1);
                }
            }

            if (getinfo) {
                std::string info = getinfo(depth, point_, n_);
                if (!info.empty()) {
                    cv::Size sz =
                            cv::getTextSize(info, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

                    cv::putText(
                            im, info, cv::Point(5, 5 + sz.height), cv::FONT_HERSHEY_PLAIN, 1,
                            cv::Scalar(255, 0, 255), 1);
                }
            }

            cv::imshow("region", im);
        }

        void DrawRect(cv::Mat &image) {  // NOLINT
            if (!show_)
                return;
            std::uint32_t n = (n_ > 1) ? n_ : 1;
            n += 1;  // outside the region
            cv::rectangle(
                    image, cv::Point(point_.x - n, point_.y - n),
                    cv::Point(point_.x + n, point_.y + n),
                    selected_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1);
        }

    private:
        std::uint32_t n_;
        bool show_;
        bool selected_;
        cv::Point point_;
    };

    void OnDepthMouseCallback(int event, int x, int y, int flags, void *userdata) {//鼠标回调事件函数 *userdata为传入的参数
        DepthRegion *region = reinterpret_cast<DepthRegion *>(userdata);//强制类型转为DepthRegion
        region->OnMouse(event, x, y, flags);//转到鼠标事件
    }

}  // namespace

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {
    ros::init (argc, argv, "pcl_create");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::PointCloud2 output;
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

    cv::namedWindow("frame");
    cv::namedWindow("depth");
    cv::namedWindow("region");

    DepthRegion depth_region(3);
    auto depth_info = [](
            const cv::Mat &depth, const cv::Point &point, const std::uint32_t &n) {
        MYNTEYE_UNUSED(depth)
        std::ostringstream os;
        os << "depth pos: [" << point.y << ", " << point.x << "]"
           << "±" << n << ", unit: mm";
        return os.str();
    };

    CVPainter painter;
    PCViewer pcviewer;

    Matrix3f k_depth_cam;//配准相关代码
    k_depth_cam << 874.63529290183885223,0,616.45473674786478568,0,874.59612482800662292,400.16292108133046668,0,0,1;
    Matrix3f k_rgb_cam;
    k_rgb_cam << 874.63529290183885223,0,616.45473674786478568,0,874.59612482800662292,400.16292108133046668,0,0,1;
    Matrix3f r_depth_rgb;
    r_depth_rgb << 1,0,0,0,1,0,0,0,1;
    RGBRectifier rectifier(k_depth_cam,k_rgb_cam,r_depth_rgb,0,0,0);
    cv::Mat  rectified;
    MAKEpoint_clould makepoint;
    Feature featurefind;
    while (true) {
        api->WaitForStreams();

        auto &&left_data = api->GetStreamData(Stream::LEFT);
        auto &&right_data = api->GetStreamData(Stream::RIGHT);
        auto &&right_data_REC = api->GetStreamData(Stream::LEFT_RECTIFIED);
        cv::Mat img;
        cv::hconcat(left_data.frame, right_data.frame, img);//图像拼接
        painter.DrawImgData(img, *left_data.img);//图片打上文字
        cv::imshow("frame", img);
        // LOG(INFO) << "left id: " << left_data.frame_id
        //     << ", right id: " << right_data.frame_id;
        auto &&disp_data = api->GetStreamData(Stream::DISPARITY_NORMALIZED);//获取深度可显示数据
        auto &&depth_data = api->GetStreamData(Stream::DEPTH);//获取深度原始数据

        int pointcount =rectifier.loadDepth(depth_data.frame);// 加载深度图，获取点云 raw_point
        rectifier.getRectifiedRGB(rectified, right_data_REC.frame, false);// 获取配准后的RGB图

//        featurefind.feature_find(rectified,rectified);//计算特征点，这个类是自己写的
        vector<KeyPoint> keypoints_1, keypoints_2;//特征点
        vector<DMatch> matches;//特征点的匹配点
        featurefind.find_feature_matches(left_data.frame,right_data.frame,keypoints_1,keypoints_2,matches);//计算特帧点
        cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;
        //-- 估计两张图像间运动
        Mat R,t;
        featurefind.pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

        imshow("right_rectified",rectified);
        auto &&points_data = api->GetStreamData(Stream::POINTS);//获取点云
        if (!points_data.frame.empty()) {
//      pcviewer.Update(points_data.frame);//CVmat图转换为点云
            // LOG(INFO) << "points id: " << points_data.frame_id;
            int count=0;
            for (int i=0;i<points_data.frame.rows;i++)
            {
                for(int j=0;j<points_data.frame.cols;j++)
                {
                    auto &&p = points_data.frame.at<cv::Point3f>(i, j);
//                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)&&p.x&&p.y&&p.z){
                    cloud.points[count].x = p.x/1000;
                    cloud.points[count].y = p.y/1000;
                    cloud.points[count].z = p.z/1000;
                    cloud.points[count].r = right_data_REC.frame.at<cv::Vec3b>(i,j)[2];
                    cloud.points[count].g = right_data_REC.frame.at<cv::Vec3b>(i,j)[1];
                    cloud.points[count].b = right_data_REC.frame.at<cv::Vec3b>(i,j)[0];
                    count++;
//                    std::cout<<p.x<<"  "<<p.y<<"  "<<p.z<<std::endl;
//                }

                }

            }
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = "odom";
            ros::Time current_time = ros::Time::now();
            output.header.stamp=current_time;
            if (ros::ok())//发布ROS消息
            {
                pcl_pub.publish(output);
                ros::spinOnce();
                loop_rate.sleep();
            }
            makepoint.makepointclould_from_xyz(rectifier.raw_point_x,rectifier.raw_point_y,
                                               rectifier.raw_point_z,pointcount,right_data_REC.frame);//从X，Y，Z生成点云
        }

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            std::cout<<"ROW"<<points_data.frame.rows<<"   COL"<<points_data.frame.cols
                     <<"recf"<<right_data_REC.frame.rows<<"   "<<right_data_REC.frame.cols
                     <<"yuan"<<right_data.frame.rows<<std::endl;
            cloud.is_dense=false;
            pcl::io::savePCDFileBinary("map_qiao_REC_LEFT_xyz_zhudongguang_no.pcd",makepoint.pointcloud_out);
//        pcl::io::savePCDFileBinary("map_qiao_REC_LEFT_cloud_3.pcd",cloud);
            break;
        }
        if (pcviewer.WasStopped()) {
            break;
        }
    }

    api->Stop(Source::VIDEO_STREAMING);
    return 0;
}

