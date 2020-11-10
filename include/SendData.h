//
// Created by wh on 2020/11/6.
//

#ifndef ARUCO_ROAD_SENDDATA_H
#define ARUCO_ROAD_SENDDATA_H

#include <pcl/segmentation/sac_segmentation.h>
class SendData {
private:
    const double pi = acos(-1.0);
public:
    SendData(){};
    ~SendData(){};
    bool sendData(pcl::ModelCoefficients::Ptr coefficients_cylinder, double X, double Y, double distance);
};


#endif //ARUCO_ROAD_SENDDATA_H
