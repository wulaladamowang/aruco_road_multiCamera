//
// Created by wh on 2020/11/3.
//

#ifndef ARUCO_ROAD_GETPOINTCLOUD_H
#define ARUCO_ROAD_GETPOINTCLOUD_H
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

class GetRoiPointCloud {
private:
    //获得点云的中心位置，三维坐标
    float distance = 0;
    float X = 0;
    float Y = 0;
//获取角点的参数
    int max_corners = 20000;
    double quality_level = 0.01;
    double min_distance = 1;
    int block_size = 5;
    bool use_harries = false;
    double k = 0.04;

public:
    GetRoiPointCloud(){};
    ~GetRoiPointCloud(){};
    bool setMaskToPointCloud(cv::Mat& input_dectect_img, pcl::PointCloud<pcl::PointXYZ>::Ptr& p_point_cloud, cv::Rect rect_roi, cv::Mat mask, sl::Mat& point_cloud);
    inline void getPosition(float& x, float& y, float& z){
        x = X;
        y = Y;
        z = distance;
    }
};


#endif //ARUCO_ROAD_GETPOINTCLOUD_H
