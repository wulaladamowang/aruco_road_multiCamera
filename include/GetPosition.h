//
// Created by wh on 2020/11/6.
//

#ifndef ARUCO_ROAD_GETPOSITION_H
#define ARUCO_ROAD_GETPOSITION_H

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

class GetPosition {
private:
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
public:
    GetPosition(){};
    ~GetPosition(){};
    bool setPointCloudToPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr p_roi_point_cloud, pcl::ModelCoefficients::Ptr& coefficients_cylinder, pcl::PointIndices::Ptr& inliers_cylinder);
};


#endif //ARUCO_ROAD_GETPOSITION_H
