#ifndef _CORRECT_HPP_
#define _CORRECT_HPP_

#include <iostream>

// 把 [图片坐标系] 转化为 [哨岗坐标系] 所需转换function
#define MAP_WIDTH   448
#define MAP_HIGH    808

// 水平翻转
void flip_horizontal(float& x) {
    if (x != -1.0) {
        x = std::fabs(MAP_WIDTH - x);
    }
}
// 垂直翻转
void flip_vertical(float& y) {
    if (y != -1.0) {
        y = std::fabs(MAP_HIGH  - y);
    }
}
// 对角翻转
void flip_diagonal(float& x, float& y) {
    flip_horizontal(x);
    flip_vertical(y);
}

// 矫正偏差
void correct_function(float& x, float& y) {
    static float watchDog   = 1789.0;
    static float carHeight  = 250.0;
    static float nicetry    = carHeight / watchDog;
    x = x * (1.0 - nicetry);
    y = y * (1.0 - nicetry);
}

// 矫正偏差
void correct_function_2(cv::Point2f& carPosition, cv::Point2f& carPositionFixed, const int& mothed) {

    // b_1 == 0  蓝方主哨岗
    // b_2 == 1  蓝方副哨岗
    // r_1 == 2  红方主哨岗
    // r_2 == 3  红方副哨岗

    // 坐标轴翻转 至 能够合适地进行坐标误差矫正
    if (mothed == 0) {
        flip_vertical(carPosition.y);
    }
    else if (mothed == 1) {
        flip_vertical(carPosition.y);
    }
    else if (mothed == 2) {
        flip_diagonal(carPosition.x, carPosition.y);
    }
    else if (mothed == 3) {
        flip_diagonal(carPosition.x, carPosition.y);
    }

    // 坐标误差矫正
    static float watchDog   = 1789.0;
    static float carHeight  = 250.0;
    static float nicetry    = carHeight / watchDog;
    carPositionFixed.x = carPosition.x * (1.0 - nicetry);
    carPositionFixed.y = carPosition.y * (1.0 - nicetry);

    // 坐标轴翻转 至 roboCar需要的坐标轴
    if (mothed == 0) {
        flip_vertical(carPosition.y);
        flip_vertical(carPositionFixed.y);
    }
    else if (mothed == 1) {
        flip_horizontal(carPosition.x);
        flip_horizontal(carPositionFixed.x);
    }
    else if (mothed == 2) {
        // 不需要做任何操作
    }
    else if (mothed == 3) {
        flip_diagonal(carPosition.x,      carPosition.y);
        flip_diagonal(carPositionFixed.x, carPositionFixed.y);
    }
    // std::swap(carPosition.x,      carPosition.y);
    // std::swap(carPositionFixed.x, carPositionFixed.y);

}

#endif  // _CORRECT_HPP_