#ifndef SENTRY_MODULE_VISUALIZE_VISUALIZE_HPP_
#define SENTRY_MODULE_VISUALIZE_VISUALIZE_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "yolov5.hpp"
#include "position.hpp"
#include "watchtower.hpp"
#include "msgs.hpp"

class Visualize {

private:
    cv::Mat background_          = ~ cv::Mat::zeros(808, 448, CV_8UC3);  // 白图(初始化变量)
    cv::Mat B3_B7_               = ~ cv::Mat::zeros(20,  100, CV_8UC3);
    cv::Mat B1_B4_B6_B9_         = ~ cv::Mat::zeros(100, 20,  CV_8UC3);
    cv::Mat B2_B8_               = ~ cv::Mat::zeros(80,  20,  CV_8UC3);
    cv::Mat B5_                  = cv::Mat::zeros(26,  26,  CV_8UC3);    // 黑图

    cv::Mat visual_map_          = ~ cv::Mat::zeros(808, 448, CV_8UC3);  // 要展示的虚拟地图(竖屏)
    
public:
    Visualize();
    ~Visualize();

    void show_img(const cv::Mat &src);
    void show_img(cv::Mat &src, std::vector<Yolo::Detection> yolo_detection);
    void show_img(cv::Mat &src, cv::Mat warp_matrix);
    void show_visual_map(std::vector<CarsInfo> &cars_info);
    void show_visual_map(WatchtowerInfo &tower_info);
    void show_watchtower_info(WatchtowerInfo &tower_info);
    void show_receive_info(ReceiveInfo &receive_info);

    // void write_vedio(cv::Mat &src, string save_vedio_path);
};

#endif // SENTRY_MODULE_VISUALIZE_VISUALIZE_HPP_