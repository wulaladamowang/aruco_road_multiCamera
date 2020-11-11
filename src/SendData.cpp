//
// Created by wh on 2020/11/6.
//

#include "SendData.h"
/*
 * 将检测的目标的位姿参数发送或者进行显示
 * coefficients: 拟合的目标物的参数，在拟合结果中，参数为轴线的法向量以及轴线上一点的坐标，由于轴线上的坐标点不确定，所以自己进行计算
 * X,Y,distance: 目标物体的位移参数，目标框中点附近的三维坐标
 */
bool SendData::sendData(pcl::ModelCoefficients::Ptr coefficients_cylinder, double X, double Y, double distance){
    std::cout << "2:" << std::this_thread::get_id << std::endl;
    double x_degree = 0.0;
    if(coefficients_cylinder->values[3] > 0)
        x_degree = acos(coefficients_cylinder->values[3])/pi*180;
    else
        x_degree = -acos(coefficients_cylinder->values[3])/pi*180;
    double y_degree = 0.0;
    if(coefficients_cylinder->values[3] > 0)
        y_degree = acos(coefficients_cylinder->values[4])/pi*180;
    else
        y_degree = -acos(coefficients_cylinder->values[4])/pi*180;
    double z_degree = 0.0;
    if(coefficients_cylinder->values[3] > 0)
        z_degree = acos(coefficients_cylinder->values[5])/pi*180;
    else
        z_degree = -acos(coefficients_cylinder->values[5])/pi*180;
    std::cout << "X: " << X << "m   Y:" << Y << "m   Z: " <<distance << "m" << std::endl;
    std::cout << "x轴:" <<  (x_degree>0?x_degree:180+x_degree) << "度"
              << "y轴:" <<  (y_degree>0?y_degree:180+y_degree) << "度"
              << "z轴:" <<  (z_degree>0?z_degree:180+z_degree) << "度" << std::endl;
    //增加条件， 判断是否发送成功
    return true;
};