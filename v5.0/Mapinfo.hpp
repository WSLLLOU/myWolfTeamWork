#ifndef _MAPINFO_HPP_
#define _MAPINFO_HPP_

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

class MapInfo
{
private:
    cv::Mat warpmatrix;     // (3, 3, CV_64FC1)  透视变换矩阵

    cv::Mat aiMap               = ~ cv::Mat::zeros(808, 448, CV_8UC3);  // 白图
    cv::Mat B3_B7               = ~ cv::Mat::zeros(20,  100, CV_8UC3);
    cv::Mat B1_B4_B6_B9         = ~ cv::Mat::zeros(100, 20,  CV_8UC3);
    cv::Mat B2_B8               = ~ cv::Mat::zeros(80,  20,  CV_8UC3);
    cv::Mat B5                  = cv::Mat::zeros(26,  26,  CV_8UC3);    // 黑图

    cv::Mat aiMapShow           = ~ cv::Mat::zeros(808, 448, CV_8UC3);  // 白图

public:
   MapInfo(cv::Mat& warpmatrix);
   ~MapInfo();
   void showMapInfo(std::vector<car>& result);
   void showTransformImg(cv::Mat& img);
   void drawCarPosition(std::vector<car>& result);
};

// 构造函数
// 给透视变换矩阵赋值 && 画一个虚拟地图
MapInfo::MapInfo(cv::Mat& warpmatrix) {
    this->warpmatrix = warpmatrix;                                  // 初始化透视变换矩阵
    // aiMap = ~aiMap;
    cv::Mat roiB3 = aiMap(cv::Rect(0,       150,      100, 20));    // cv::Rect(左上角的点 和 宽高)
    cv::Mat roiB7 = aiMap(cv::Rect(448-100, 808-170,  100, 20));
    cv::absdiff(roiB3, B3_B7, roiB3);                               // 相减，取绝对值
    cv::absdiff(roiB7, B3_B7, roiB7);

    cv::Mat roiB1 = aiMap(cv::Rect(448-120, 0,        20,  100));
    cv::Mat roiB4 = aiMap(cv::Rect(100,     808-100,  20,  100));
    cv::Mat roiB6 = aiMap(cv::Rect(93,      354,      20,  100));   // 93  -> 93.5
    cv::Mat roiB9 = aiMap(cv::Rect(335,     354,      20,  100));   // 335 -> 334.5
    cv::absdiff(roiB1, B1_B4_B6_B9, roiB1);
    cv::absdiff(roiB4, B1_B4_B6_B9, roiB4);
    cv::absdiff(roiB6, B1_B4_B6_B9, roiB6);
    cv::absdiff(roiB9, B1_B4_B6_B9, roiB9);

    cv::Mat roiB2 = aiMap(cv::Rect(214,     150,      20,  80));
    cv::Mat roiB8 = aiMap(cv::Rect(214,     578,      20,  80));
    cv::absdiff(roiB2, B2_B8, roiB2);
    cv::absdiff(roiB8, B2_B8, roiB8);

    cv::Mat roiB5 = aiMap(cv::Rect(224-13,  404-13,   26,  26));
    // 在B5上画个旋转矩形
    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(13,13), cv::Size2f(18, 18), 45);
    cv::Point2f vertices2f[4];                                      // 定义4个点的数组
    rRect.points(vertices2f);                                       // 将四个点存储到 `vertices` 数组中
    cv::Point   vertices[4];
    for (int i = 0; i < 4; ++i) {
        vertices[i] = vertices2f[i];
    }
    cv::fillConvexPoly(B5, vertices, 4, cv::Scalar(255, 255, 255));
    cv::absdiff(roiB5, B5, roiB5);

    aiMap.copyTo(aiMapShow);
}

MapInfo::~MapInfo() {
    
}

// 显示 透视变换 后的图像
void MapInfo::showTransformImg(cv::Mat& img) {
    static cv::Mat result;
    cv::warpPerspective(img, result, this->warpmatrix, cv::Size(448, 808),cv::INTER_LINEAR); //result.size(),
    cv::imshow("result", result);
}

void MapInfo::showMapInfo(std::vector<car>& result) {
    drawCarPosition(result);
    cv::imshow("map", aiMapShow);
    aiMap.copyTo(aiMapShow);
}

void MapInfo::drawCarPosition(std::vector<car>& result) {
    for (car& p: result) {
        cv::circle(aiMapShow, cv::Point(p.carPosition.x, p.carPosition.y),              25, cv::Scalar(0, 255, 0)); // 矫正前的
        cv::circle(aiMapShow, cv::Point(p.carPositionFixed.x, p.carPositionFixed.y),    25, cv::Scalar(255, 0, 0)); // 矫正后的
    }
}



#endif  // _MAPINFO_HPP_