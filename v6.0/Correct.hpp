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

#endif  // _CORRECT_HPP_