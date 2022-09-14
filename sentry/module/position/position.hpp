#ifndef SENTRY_MODULE_POSITION_POSITION_HPP_
#define SENTRY_MODULE_POSITION_POSITION_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include "yolov5.hpp"

// 配置参数
struct FixPositionConfig{
    int   fix_position_method;
    float watchtower_high;
    float car_half_high;
    float offset_x;
    float offset_y;

    FixPositionConfig(int fpm_, float wh_, float chh_, float ox_, float oy_) : fix_position_method(fpm_), watchtower_high(wh_), car_half_high(chh_), offset_x(ox_), offset_y(oy_) {}
} ;

typedef struct {
    cv::Rect    img_rect;           // `图像`上的车车下半身矩形框 --> 用于寻找装甲板存在的ROI
    cv::Point2f position;           // 以B4为原点, 短边为x, 长边为y 的 `世界地图` 上的坐标点
    int         color;              // 0蓝 / 1红 / 2黑
    int         num;                // 1 / 2
} CarsInfo;

typedef struct {
    cv::Point   img_armor_center;   // `图像`上的装甲板中心点
    int         color;              // 0蓝 / 1红 / 2黑
    int         num;                // 1 / 2
} ArmorsInfo;


class Position {
    private:
        const float kMapWidth = 448;
        const float kMapHigh  = 808;

        // fix_position_method_ 数据矫正函数模式
        // 0  蓝方[主哨岗] D1
        // 1  蓝方[副哨岗] D3
        // 2  红方[主哨岗] D4
        // 3  红方[副哨岗] D2
        int                         fix_position_method_;
        cv::Mat                     warp_matrix_;           // (3, 3, CV_64FC1)  透视变换矩阵
        cv::Mat                     tool_img_;              // 工具图img
        std::vector<CarsInfo>       cars_info_;             // 存放每张图像的所有车车信息 <--(车车的颜色需要依靠armors_info_去填充)
        std::vector<ArmorsInfo>     armors_info_;           // 存放每张图像的所有装甲板信息

    private:
        void        yoloDetection2CarsInfo(std::vector<Yolo::Detection> &predicts);
        cv::Point2f getWarpPosition(const cv::Point2f &ptOrigin);
        cv::Point2f getCenterPoint(const cv::Rect &rect);       // 求矩形中心点
        cv::Point2f getCenterPoint(const cv::Point2f pts[]);    // 求旋转矩形四点中心
        
        // 水平翻转
        inline void flipHorizontal(float& x)            { if (x != -1.0) { x = std::fabs(this->kMapWidth - x); } }
        // 垂直翻转
        inline void flipVertical(float& y)              { if (y != -1.0) { y = std::fabs(this->kMapHigh - y); } }
        // 对角翻转
        inline void flipDiagonal(float& x, float& y)   { flipHorizontal(x); flipVertical(y); }
        // 
        void        fixCarsPosition(cv::Point2f& car_position);

    public:
        Position(int fix_position_method, cv::Mat warp_matrix, cv::Mat src_img);
        ~Position();

        std::vector<CarsInfo> get_cars_info(std::vector<Yolo::Detection> &predicts);
};

#endif  // SENTRY_MODULE_POSITION_POSITION_HPP_