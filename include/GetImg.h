//
// Created by wang on 2020/10/16.
//

#ifndef ARUCO_ROAD_GETIMG_H
#define ARUCO_ROAD_GETIMG_H

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <iostream>
/*
 * 开启相机并获得图像
 */
class GetImg{
public:
    GetImg();
    // GetImg(char* file);
    ~GetImg(){
        // if(is_opened)
        //     zed.close();
    }
    friend cv::Mat slMat2cvMat(sl::Mat& input);
    //程序未加锁img_left_cv，所以抓取和提取不应当分线程, 抓取图像
    void grubImage(sl::Camera& zed_input);
    // 获得图像或点云
    inline void getImage(char* left_or_right, cv::Mat& output_img) const
    {
        if("left" == left_or_right)
            output_img = img_left_cv;
        else
            output_img =  img_right_cv;
    }
    inline void getPointCloud(sl::Mat& output_point_cloud) const
    {
        output_point_cloud = point_cloud;
    }
    inline void getTimestamp(sl::Timestamp& input_timestamp){
        input_timestamp = ts_g;
    }
private:
    bool is_opened = false;
    sl::Camera zed;
    ///获得图像，zed格式及opencv格式
    sl::Mat img_left_zed;
    cv::Mat img_left_cv;
    sl::Mat img_right_zed;
    cv::Mat img_right_cv;
    sl::Mat point_cloud;
    sl::Timestamp ts_g;
};

#endif //ARUCO_ROAD_GETIMG_H
