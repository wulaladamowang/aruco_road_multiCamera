//
// Created by wang on 2020/10/16.
//

#include "GetImg.h"
/*
 * 辅助坐标转换函数，将ZED格式转化为opencv的识别格式
 */
cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}
/*
 * 初始化相机
 */
GetImg::GetImg() {
    if(is_opened)
        zed.close();
    is_opened = false;
    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "ZED打开失败: " << toString(err).c_str() << std::endl;
        zed.close();
        is_opened = false; // Quit if an error occurred
    } else
    {
        is_opened = true;
        std::cout << "ZED打开成功" << std::endl;
    }
}
/*
 *
 */
GetImg::GetImg(char *file) {
    if(is_opened)
        zed.close();
    sl::InitParameters init_params;
    init_params.input.setFromSVOFile(file);
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "文件打开失败: " << toString(err).c_str() << std::endl;
        zed.close();
        is_opened = false;
    } else
    {
        is_opened = true;
        std::cout << "文件打开成功" << std::endl;
    }
}
/*
 * 抓取图像
 */
void GetImg::grubImage()
{
    if (zed.grab(sl::SENSING_MODE::STANDARD) == sl::ERROR_CODE::SUCCESS){
        zed.retrieveImage(img_left_zed, sl::VIEW::LEFT, sl::MEM::CPU);
        img_left_cv = slMat2cvMat(img_left_zed);
        cv::cvtColor(img_left_cv,img_left_cv,cv::COLOR_BGRA2BGR);
//        zed.retrieveImage(img_right_zed, sl::VIEW::RIGHT, sl::MEM::CPU);
//        img_right_cv = slMat2cvMat(img_right_zed);
//        cv::cvtColor(img_right_cv,img_right_cv,cv::COLOR_BGRA2BGR);
        zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
    }
    else
        sl::sleep_ms(1);
};