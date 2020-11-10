#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/time.h>

#include "GetImg.h"
#include "GetRoi.h"
#include "GetMarker.h"
#include "GetRoiPointCloud.h"
#include "GetPosition.h"
#include "SendData.h"


int main() {

    cv::Mat img_left;
    sl::Mat point_cloud;
    GetImg getImg;

    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    marker_corners.reserve(30);
    marker_ids.reserve(30);
    GetMarker getMarker;
    GetRoi getRoi(cv::Size(1920, 1080));

    float distance = 0;
    float X = 0;
    float Y = 0;

    GetRoiPointCloud getRoiPointCloud;



    cv::Mat mask;
    cv::Rect rect_roi;
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::CloudViewer viewer("Point Cloud");
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    GetPosition getPosition;

    SendData sendData;
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);


    char key = ' ';
    while(key!='q'){
        getImg.grubImage();
        getImg.getImage("left",img_left);
        getImg.getPointCloud(point_cloud);

        getMarker.detectMarkers(img_left, marker_corners, marker_ids);

        if(marker_corners.size()>0){
            getRoi.setCornerToMask(marker_corners, marker_ids);
            getRoi.getMask(mask);
            getRoi.getMaskRect(rect_roi);

            getRoiPointCloud.setMaskToPointCloud(img_left, p_point_cloud, rect_roi, mask, point_cloud);
            getRoiPointCloud.getPosition(X, Y, distance);
            viewer.showCloud(p_point_cloud);
            getPosition.setPointCloudToPosition(p_point_cloud, coefficients_cylinder, inliers_cylinder);

            sendData.sendData(coefficients_cylinder, X, Y, distance);
            gettimeofday(&t2, NULL);
            std::cout << " 时间："<< ((t2.tv_sec-t1.tv_sec) * 1000 + (t2.tv_usec-t1.tv_usec)/1000) << "毫秒" << std::endl;
            gettimeofday(&t1, NULL);
        }else
        {
            X = 0;
            Y = 0;
            distance = 0;
        }
        key = cv::waitKey(10);
        cv::imshow("img_left", img_left);
    }

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
