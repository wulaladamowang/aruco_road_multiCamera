//
// Created by wh on 2020/11/6.
//

#include "GetRoiPointCloud.h"
/*
 * input_detect_img: 用来提取角点的原图
 * p_point_cloud: 输入输出参数， 保存提取的点云
 * rect_roi: 用于在原图上选择提取角点的区域，为正矩形，用于检测角点的图像分辨率越小，其检测速度越快
 * mask: 识别到的目标区域的位置，其为旋转的最小矩形框，目标区域被置为1
 * point_cloud: 点云图，每个Input_detect_img上的点对应一个三维坐标
 */
bool GetRoiPointCloud::setMaskToPointCloud(cv::Mat& input_dectect_img, pcl::PointCloud<pcl::PointXYZ>::Ptr& p_point_cloud, cv::Rect rect_roi, cv::Mat mask, sl::Mat& point_cloud)
{
    cv::Mat img_gray = cv::Mat::zeros(rect_roi.width, rect_roi.height, CV_8UC1);
    cv::Mat roi_rect = input_dectect_img(rect_roi).clone();
    cv::cvtColor(roi_rect, img_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img_gray, corners, max_corners, quality_level,
                            min_distance, cv::Mat(), block_size, use_harries, k);
    ///存储检测的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    p_pcl_point_cloud->points.resize(corners.size()+20);
    auto it = p_pcl_point_cloud->points.begin();
///通过检测的roi的中点做后续的点云筛选标准， 同时默认检测的位置即为此位置
    distance = 0;
    X = 0;
    Y = 0;
    int center_x = rect_roi.x + rect_roi.width/2;
    int center_y = rect_roi.y + rect_roi.height/2;
    int count = 0;
    for(int i=-15;i<15;i+=1){
        for(int j=-15;j<15;j+=1){
            sl::float4 point_3d;
            point_cloud.getValue(center_x+i, center_y+j, &point_3d);
            if(isValidMeasure(point_3d.z) && (point_3d.z<10) && (point_3d.z>0.15)){
                count++;
                distance+=point_3d.z;
                X+=point_3d.x;
                Y+=point_3d.y;
            }
        }
    }
    distance = distance/count;
    X = X / count;
    Y = Y / count;

    //如果检测的距离不为有效参数，则将检测的阈值设置为目标识别范围;
    float min , max;
    bool isValidDistance = isValidMeasure(distance);
    if(!isValidDistance)
    {
        min = 0.15;
        max = 10;
        X = 0;
        Y = 0;
        distance = 0;
        count = 0;
    }else{
            min = distance - 0.5;
            max = distance + 0.5;//用于试验的目标物的长度大约为1M，半径约为0.3M
    }
    for(auto& corner : corners){
        int x = corner.x + rect_roi.x;
        int y = corner.y + rect_roi.y;

        //由于检测点的点云不一定存在，则通过提取十字的五个点中的一个作为提取结果
        for(int i=-1;i<=1;i++){
            for(int j=-1;j<=1;j++){
                if((x+i)<input_dectect_img.cols && (x+i)>=0 &&
                   (y+j)<input_dectect_img.rows && (y+j)>=0 &&
                   (0!=mask.ptr(y)[x]))
                {
                    
                    sl::float4 point_3d;
                    point_cloud.getValue(x+i, y+j, &point_3d);
            
                    float z = point_3d.z;
                    if(!isValidMeasure(z) || max<z || min>z) {
                    }else{
                        i = 2;
                        j = 2;
                        it->x = point_3d.x;
                        it->y = point_3d.y;
                        it->z = point_3d.z;
                        ++it;
                        if(!isValidDistance){
                            count++;
                            X += point_3d.x;
                            Y += point_3d.y;
                            distance += point_3d.z;
                        }
                    }
                }
            }
        }
    }
    if(!isValidDistance)
    {
        X = X/count;
        Y = Y/count;
        distance = distance/count;
    }
    p_point_cloud = p_pcl_point_cloud;
    //判断提取的数量，当提取数量过少时，后续点云拟合效果不好
    return p_pcl_point_cloud->size()>50?true: false;
};