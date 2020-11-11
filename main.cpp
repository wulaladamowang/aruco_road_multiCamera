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

void zed_process(sl::Camera& zed, int& n, cv::Mat& img_left, bool& run, sl::Timestamp& ts);

int main(int argc, char** argv){
    sl::InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;

    std::vector<sl::DeviceProperties> devList = sl::Camera::getDeviceList();
    int nb_detected_zed = devList.size();

    for (int z = 0; z < nb_detected_zed; z++) {
		std::cout << "ID : " << devList[z].id << " ,model : " << devList[z].camera_model << " , S/N : " << devList[z].serial_number << " , state : "<<devList[z].camera_state<<std::endl;
	}
    if (nb_detected_zed == 0) {
        cout << "No ZED Detected, exit program" << endl;
        return EXIT_FAILURE;
    }
    std::cout << nb_detected_zed << " ZED Detected" << std::endl;
    std::vector<sl::Camera> zeds(nb_detected_zed);
    for (int z = 0; z < nb_detected_zed; z++) {
        init_parameters.input.setFromCameraID(z);
        sl::ERROR_CODE err = zeds[z].open(init_parameters);
        if (err == sl::ERROR_CODE::SUCCESS) {
            auto cam_info = zeds[z].getCameraInformation();
            cout << cam_info.camera_model << ", ID: " << z << ", SN: " << cam_info.serial_number << " Opened" << endl;
        } else {
            cout << "ZED ID:" << z << " Error: " << err << endl;
            zeds[z].close();
        }
    }
    bool run = true;
    std::vector<std::thread> thread_pool(nb_detected_zed); // compute threads
    std::vector<cv::Mat> images_lr(nb_detected_zed); // display images
    std::vector<std::string> wnd_names(nb_detected_zed); // display windows names
    std::vector<sl::Timestamp> images_ts(nb_detected_zed); // images timestamps
    for (int z = 0; z < nb_detected_zed; z++)
        if (zeds[z].isOpened()) {
            // create an image to store Left+Depth image
            images_lr[z] = cv::Mat::ones(1920, 1080, CV_8UC4);
            // camera acquisition thread
            thread_pool[z] = std::thread(zed_process, std::ref(zeds[z]), std::ref(z),std::ref(images_lr[z]), std::ref(run), std::ref(images_ts[z]));
            // create windows for display
            wnd_names[z] = "ZED ID: " + std::to_string(z);
            cv::namedWindow(wnd_names[z]);
        }
    std::vector<sl::Timestamp> last_ts(nb_detected_zed, 0); // use to detect new images
    char key = ' ';
    // Loop until 'Esc' is pressed
    while (key != 'q') {
        // Resize and show images
        for (int z = 0; z < nb_detected_zed; z++) {
            if (images_ts[z] > last_ts[z]) { // if the current timestamp is newer it is a new image
                cv::imshow(wnd_names[z], images_lr[z]);
                last_ts[z] = images_ts[z];
            }
        }

        key = cv::waitKey(10);
    }

    run = false;

    // Wait for every thread to be stopped
    for (int z = 0; z < nb_detected_zed; z++)
        if (zeds[z].isOpened()) 
            thread_pool[z].join();
    return 0;
}
void zed_process(sl::Camera& zed, int& n, cv::Mat& img_left, bool& run, sl::Timestamp& ts){
    GetImg getImg;
    sl::Mat point_cloud;
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

    std::string str = "Point_cloud" + std::to_string(n);
    pcl::visualization::CloudViewer viewer(str);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    GetPosition getPosition;

    SendData sendData;

    while (run){
        getImg.grubImage(zed);
        getImg.getImage("left",img_left);
        getImg.getPointCloud(point_cloud);
        getImg.getTimestamp(ts);

        getMarker.detectMarkers(img_left, marker_corners, marker_ids);

        if(marker_corners.size()>0){
            getRoi.setCornerToMask(marker_corners, marker_ids);
            getRoi.getMask(mask);
            getRoi.getMaskRect(rect_roi);
            
            getRoiPointCloud.setMaskToPointCloud(img_left, p_point_cloud, rect_roi, mask, point_cloud);
            getRoiPointCloud.getPosition(X, Y, distance);
            viewer.showCloud(p_point_cloud);
            getPosition.setPointCloudToPosition(p_point_cloud, coefficients_cylinder, inliers_cylinder);
        }else
        {
            X = 0;
            Y = 0;
            distance = 0;
        }
        sendData.sendData(coefficients_cylinder, X, Y, distance);
    }
    zed.close();
}

// int main() {

//     cv::Mat img_left;
//     sl::Mat point_cloud;
//     GetImg getImg;

//     std::vector<std::vector<cv::Point2f>> marker_corners;
//     std::vector<int> marker_ids;
//     marker_corners.reserve(30);
//     marker_ids.reserve(30);
//     GetMarker getMarker;
//     GetRoi getRoi(cv::Size(1920, 1080));

//     float distance = 0;
//     float X = 0;
//     float Y = 0;

//     GetRoiPointCloud getRoiPointCloud;



//     cv::Mat mask;
//     cv::Rect rect_roi;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr p_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::visualization::CloudViewer viewer("Point Cloud");
//     pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
//     GetPosition getPosition;

//     SendData sendData;
//     // struct timeval t1, t2, t3, t4, t5, t6, t7, ta, tb;
//     // gettimeofday(&ta, NULL);


//     char key = ' ';
//     while(key!='q'){
//         // gettimeofday(&tb, NULL);
//         // std::cout << " 时间total："<< ((tb.tv_sec-ta.tv_sec) * 1000 + (tb.tv_usec-ta.tv_usec)/1000) << "毫秒" << std::endl;
//         // gettimeofday(&ta, NULL);
//         // gettimeofday(&t1, NULL);
//         getImg.grubImage();
//         getImg.getImage("left",img_left);
//         getImg.getPointCloud(point_cloud);
//         // gettimeofday(&t2, NULL);
//         // std::cout << " 时间1："<< ((t2.tv_sec-t1.tv_sec) * 1000 + (t2.tv_usec-t1.tv_usec)/1000) << "毫秒" << std::endl;
//         getMarker.detectMarkers(img_left, marker_corners, marker_ids);
//         // gettimeofday(&t3, NULL);
//         // std::cout << " 时间2："<< ((t3.tv_sec-t2.tv_sec) * 1000 + (t3.tv_usec-t2.tv_usec)/1000) << "毫秒" << std::endl;

//         if(marker_corners.size()>0){

//             getRoi.setCornerToMask(marker_corners, marker_ids);
//             getRoi.getMask(mask);
//             getRoi.getMaskRect(rect_roi);
//             // gettimeofday(&t4, NULL);
//             // std::cout << " 时间3："<< ((t4.tv_sec-t3.tv_sec) * 1000 + (t4.tv_usec-t3.tv_usec)/1000) << "毫秒" << std::endl;

//             getRoiPointCloud.setMaskToPointCloud(img_left, p_point_cloud, rect_roi, mask, point_cloud);
//             getRoiPointCloud.getPosition(X, Y, distance);
//             viewer.showCloud(p_point_cloud);
//             // gettimeofday(&t5, NULL);
//             // std::cout << " 时间4："<< ((t5.tv_sec-t4.tv_sec) * 1000 + (t5.tv_usec-t4.tv_usec)/1000) << "毫秒" << std::endl;
//             getPosition.setPointCloudToPosition(p_point_cloud, coefficients_cylinder, inliers_cylinder);
//             // gettimeofday(&t6, NULL);
//             // std::cout << " 时间5："<< ((t6.tv_sec-t5.tv_sec) * 1000 + (t6.tv_usec-t5.tv_usec)/1000) << "毫秒" << std::endl;
             
 
//         }else
//         {
//             X = 0;
//             Y = 0;
//             distance = 0;
//         }
//         sendData.sendData(coefficients_cylinder, X, Y, distance);
//         // gettimeofday(&t7, NULL);
//         // std::cout << " 时间6："<< ((t7.tv_sec-t6.tv_sec) * 1000 + (t7.tv_usec-t6.tv_usec)/1000) << "毫秒" << std::endl;
//         key = cv::waitKey(10);
//         cv::imshow("img_left", img_left);
//     }

//     std::cout << "Finish, bye~" << std::endl;
//     return 0;
// }
