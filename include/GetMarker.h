//
// Created by wh on 2020/11/3.
//

#ifndef ARUCO_ROAD_GETMARKER_H
#define ARUCO_ROAD_GETMARKER_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
/*
 * 通过引用将图像aruco marker 检测并返回，注意用于识别的marker，要与字典中的相匹配;若未检测到，则返回false
 * img: 用来检测aruco marker 的图像
 * marker_corners: 用来保存检测到的marker 的角点信息
 * marker_ids: 用来保存检测到的marker 的id
 */
class GetMarker {
public:
    GetMarker(){};
    bool detectMarkers(cv::Mat img, std::vector<std::vector<cv::Point2f>>& marker_corners, std::vector<int>& marker_ids){
        marker_corners.clear();
        marker_ids.clear();
        cv::aruco::detectMarkers(img, c_dictionary, marker_corners, marker_ids);
        return marker_ids.size();
    };
    ~GetMarker(){};
private:
    //贴码的规则
    const cv::Ptr<cv::aruco::Dictionary> c_dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_4X4_50);//DICT_6X6_1000
};

#endif //ARUCO_ROAD_GETMARKER_H
