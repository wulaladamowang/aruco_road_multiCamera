//
// Created by wang on 2020/10/18.
//

#ifndef ARUCO_ROAD_GETROI_H
#define ARUCO_ROAD_GETROI_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <sl/Camera.hpp>

/*
 * 初始化一个用作mask的Mat,通过检测的角点将相应的roi区域置为255,同时确定正矩形可以包围roi
 * input_mask_size: mask的大小，与原图应当一致， 用来标志roi的区域，mask，用来保存获得的mask, 检测到的roi区域被置为255
 * marker_corners: 保存的原图的检测的到角点的位置信息
 * marker_ids: 保存的原图的检测的角点的id信息
 */
class GetRoi {
private:
    cv::Mat mask;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
public:
    GetRoi(cv::Size input_mask_size){mask = cv::Mat::zeros(input_mask_size, CV_8UC1);};
    bool setCornerToMask(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> marker_ids) ;//获得mask位置
    void getMask(cv::Mat& output_mask){output_mask = mask.clone();};
    void getMaskRect(cv::Rect& output_roi_rect) {
        output_roi_rect = cv::Rect(min_x, min_y, max_x-min_x, max_y-min_y);
    };
    //辅助函数
    float relativeDis(cv::Vec4f line_para, cv::Point2f point);
};
#endif //ARUCO_ROAD_GETROI_H
