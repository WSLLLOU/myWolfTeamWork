#ifndef TRTX_YOLOV5_CAR_TEST_HPP_
#define TRTX_YOLOV5_CAR_TEST_HPP_

#include <iostream>
#include <vector>
#include "common.hpp"
#include "map_info.hpp"

typedef struct Position {			// 车车坐标点
    float x;
    float y;
} Pos;

struct CarPositionSend {			// 套接字内容
    Pos blue1;
    Pos blue2;
    Pos red1;
    Pos red2;

    Pos gray1;
    Pos gray2;
    Pos gray3;
    Pos gray4;
} socketInfo;


typedef struct
{
    cv::Rect    img_r;
    cv::Point2f carPosition;
    int         color;      // 0 gray, 1 blue, 2 red, -1 ???
} car;


typedef struct
{
    cv::Point   img_center;
    int         color;      // 0 gray, 1 blue, 2 red, -1 ???
} armor;

// float watchDog  = 1768.0;
// float carHeight = 250.0;
// float nicetry = carHeight / watchDog;

std::vector<car>    allCar;
std::vector<armor>  allArmor;

// rect.contains(cv::Point(x, y));              //返回布尔变量，判断rect是否包含Point(x, y)点
// center (r.x+r.width/2, r.y+r.height/2)

void classify(std::vector<Yolo::Detection> res, cv::Mat img, cv::Mat& warpmatrix) {
    // 分类数据
    // 分到 allCar 和 allArmor
    for (size_t j = 0; j < res.size(); j++) { // res.size() 该图检测到多少个class
        car     temp_car;
        armor   temp_armor;
        cv::Rect r = get_rect(img, res[j].bbox);

        if (res[j].class_id == 0) {  // 0 'car'
            temp_car.img_r          = r;
            temp_car.color          = -1;
            temp_car.carPosition    = getTargetPoint(cv::Point2f(r.x+r.width/2, r.y+r.height/2), warpmatrix);
            // temp_car.carPosition.x = temp_car.carPosition.x - (nicetry * temp_car.carPosition.x);
            // temp_car.carPosition.y = temp_car.carPosition.y - (nicetry * temp_car.carPosition.y);
            allCar.push_back(temp_car);
        }
        else if (res[j].class_id == 1 || res[j].class_id == 2) {    // 1 'armor_1_blue', 2 'armor_2_blue'
            temp_armor.img_center   = cv::Point(r.x+r.width/2, r.y+r.height/2);
            temp_armor.color        = 1;
            allArmor.push_back(temp_armor);

        }
        else if (res[j].class_id == 3 || res[j].class_id == 4) {    // 3 'armor_1_red',  4 'armor_2_red'
            temp_armor.img_center   = cv::Point(r.x+r.width/2, r.y+r.height/2);
            temp_armor.color        = 2;
            allArmor.push_back(temp_armor);
        }
        else if (res[j].class_id == 5) {                            // 5 'armor_gray'
            temp_armor.img_center   = cv::Point(r.x+r.width/2, r.y+r.height/2);
            temp_armor.color        = 0;
            allArmor.push_back(temp_armor);
        }
    }
}

void carColorProcess() {
    for (auto i = allCar.begin(); i != allCar.end(); i++) {
        for (auto j = allArmor.begin(); j != allArmor.end(); j++) {
            if( (*i).img_r.contains((*j).img_center) ) {
                (*i).color = (*j).color;
                break;
            }
        }
    }
}

void showCarInfo() {
    for (auto i = allCar.begin(); i != allCar.end(); i++) {
        if ((*i).color == 1) {
            std::cout << "car_color:        blue" << std::endl;
        }
        else if ((*i).color == 2) {
            std::cout << "car_color:        red" << std::endl;
        }
        else if ((*i).color == 0) {
            std::cout << "car_color:        gray" << std::endl;
        }
        else if ((*i).color == -1) {
            std::cout << "car_color:        I don't know" << std::endl;
        }
        std::cout << "carPosition_x:    " << (*i).carPosition.x << std::endl;
        std::cout << "carPosition_y:    " << (*i).carPosition.y << std::endl;
        std::cout << std::endl;
    }
    allCar.clear();
    allArmor.clear();
}


#endif  // TRTX_YOLOV5_CAR_TEST_HPP_