//
// Created by wh on 2020/11/6.
//
#include "GetPosition.h"
/*
 * 用来进行将点云拟合为圆柱
 * p_roi_point_cloud: 提取到的目标区域的点云
 * coefficients_cylinder：拟合获得的圆柱的参数
 * inliers_cylinder: 用于拟合的三维点可以通过此参数提取出来
 */
bool GetPosition::setPointCloudToPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr p_roi_point_cloud, pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointIndices::Ptr& inliers_cylinder){
    ///计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud(p_roi_point_cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);//设置分割模型为圆柱
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits(0.10, 0.20);
    seg.setInputCloud(p_roi_point_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //可以增加条件用来判断是否满足条件，例如，两次获得坐标的差距应该满足条件
    return true;
}