#ifndef _MONITORING_HPP_
#define _MONITORING_HPP_

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include "Correct.hpp"
#include <algorithm>

typedef struct
{
    cv::Rect    img_r;              // `图像`上的车车下半身矩形框 --> 用于寻找装甲板的存在roi
    cv::Point2f carPosition;        // 矫正前的`世界地图`上的坐标点
    cv::Point2f carPositionFixed;   // 矫正后的`世界地图`上的坐标点
    int         color;              // 0蓝 1红 2黑 3紫
    int         num;                // 1 / 2
} car;

typedef struct
{
    cv::Point   img_center;         // `图像`上的装甲板中心点
    int         color;              // 0蓝 1红 2黑
    int         num;                // 1 / 2
} armor;

// 求旋转矩形四点中心
cv::Point2f opt4ToCenter(const cv::Point2f pts[]) {
    static cv::Point2f center;
    center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4.0;
    center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4.0;
    return center;
}

// 求矩形中心点
cv::Point getCenterPoint(const cv::Rect &rect)
{
    static cv::Point2f center;
    center.x = rect.x + cvRound(rect.width  / 2.0);
    center.y = rect.y + cvRound(rect.height / 2.0);
    return center;
}

class Monitoring {
    private:
        cv::Mat                 warpmatrix;     // (3, 3, CV_64FC1)  透视变换矩阵
        std::vector<car>        allCar;         // 存放每张图像的所有车车信息 <--(车车的颜色需要依靠allArmor去填充)
        std::vector<armor>      allArmor;       // 存放每张图像的所有装甲板信息 

    public:
        friend void flip_horizontal(float& x);
        friend void flip_vertical(float& y);
        friend void flip_diagonal(float& x, float& y);
        friend void correct_function(float& x, float& y);

        Monitoring(cv::Mat& warpmatrix);
        ~Monitoring();
        cv::Point2f     getTargetPoint (const cv::Point& ptOrigin);
        void            analyseData(std::vector<Yolo::Detection>& predicts, std::vector<car>& allCar, std::vector<armor>& allArmor, cv::Mat& img, cv::Mat& warpmatrix);
        void            run (std::vector<Yolo::Detection>& rtxCars, cv::Mat& img, std::vector<car>& result);
        void            fixCarPosition(std::vector<car>& allCar);
        inline cv::Mat& getmatrix()  { return this->warpmatrix; }
};

// 构造函数
Monitoring::Monitoring(cv::Mat& warpmatrix) {
    this->warpmatrix = warpmatrix;
}

Monitoring::~Monitoring() {
    
}

// 传一中心点 透视
cv::Point2f Monitoring::getTargetPoint(const cv::Point& ptOrigin) {
	cv::Mat_<double> matPt(3, 1);
	matPt(0, 0) = ptOrigin.x;
	matPt(1, 0) = ptOrigin.y;
	matPt(2, 0) = 1;
	cv::Mat matPtView = this->warpmatrix * matPt;
	double x = matPtView.at<double>(0, 0);
	double y = matPtView.at<double>(1, 0);
	double z = matPtView.at<double>(2, 0);
    
	return cv::Point2f(x * 1.0 / z, y * 1.0 / z);
}


//自定义排序函数  
bool SortByConf(const Yolo::Detection& predicts_1, const Yolo::Detection& predicts_2)   //注意：本函数的参数的类型一定要与vector中元素的类型一致  
{   
    return predicts_1.conf < predicts_2.conf;  //升序排列
}

// 分析参数 到 --> allCar 
void Monitoring::analyseData(std::vector<Yolo::Detection>& predicts, std::vector<car>& allCar, std::vector<armor>& allArmor, cv::Mat& img, cv::Mat& warpmatrix) {
    std::sort(predicts.begin(), predicts.end(), SortByConf);    // 按置信度升序
    // 分类数据（分析）
    // 分到 allCar 和 allArmor
    for (Yolo::Detection &predict : predicts) {   // predicts.size() 该图检测到多少个物体
        // car
        static car      temp_car;
        static cv::Rect temp_r;
        temp_r  = get_rect(img, predict.bbox);
        // armor
        static armor       temp_armor;
        static cv::Point2f img_armor_center;

        if (predict.class_id == 0) {             // 0 'car'
            temp_car.carPosition        = getTargetPoint(cv::Point2f(temp_r.x+temp_r.width/2, temp_r.y+temp_r.height/2));   // 透视变换矩形 对 车体检测框中心点 进行坐标转化 
            temp_car.carPositionFixed   = cv::Point2f(-1.0, -1.0);
            temp_car.color              = -1;   // 初始化 -1
            temp_car.num                = -1;   // 初始化 -1
            // 修改矩形框的大小，基于左下角垂直缩小一半，便于分类装甲板的归属
            temp_r                      = temp_r + cv::Point(0, temp_r.height/2);       //平移，左上顶点的 `x坐标`不变，`y坐标` +temp_r.height/2
            temp_r                      = temp_r + cv::Size (0, -temp_r.height/2);      //缩放，左上顶点不变，宽度不变，高度减半
            temp_car.img_r              = temp_r;
            // 
            allCar.push_back(temp_car);
        }
        else {
            // img_center
            img_armor_center = getCenterPoint(get_rect(img, predict.bbox));
            temp_armor.img_center = img_armor_center;

            // color && armor_num
            // color // 0蓝 1红 2黑
            // armor // 1 / 2
            if (predict.class_id == 1) {        // armor_1_blue
                temp_armor.color    = 0;
                temp_armor.num      = 1;
            }
            else if (predict.class_id == 2) {   // armor_2_blue
                temp_armor.color    = 0;
                temp_armor.num      = 2;
            }
            else if (predict.class_id == 3) {   // armor_1_red
                temp_armor.color    = 1;
                temp_armor.num      = 1;
            }
            else if (predict.class_id == 4) {   // armor_2_red
                temp_armor.color    = 1;
                temp_armor.num      = 2;
            }
            else if (predict.class_id == 5) {   // armor_gray
                temp_armor.color    = 2;
                temp_armor.num      = -1;       // 主不关心，死车号数
            }

            allArmor.push_back(temp_armor);
        }
    }

    // rect.contains(cv::Point(x, y));              //返回布尔变量，判断rect是否包含Point(x, y)点
    // center (r.x+r.width/2, r.y+r.height/2)
    // 判断 [car 的矩形框[] 中是否有 [armor 的中心点] 存在
    for (auto i = allCar.begin(); i != allCar.end(); i++) {
        for (auto j = allArmor.begin(); j != allArmor.end(); j++) {
            if( (*i).img_r.contains((*j).img_center) ) {
                (*i).color = (*j).color;
                (*i).num = (*j).num;
                break;
            }
        }
    }
}

void Monitoring::fixCarPosition(std::vector<car>& allCar) {
    static float fixPosX;
    static float fixPosY;
    for (auto &Car : allCar) {

        // fixPosX = Car.carPosition.x;
        // fixPosY = Car.carPosition.y;
        
        // flip_vertical(Car.carPosition.y);
        
        // flip_vertical(fixPosY);                 // 垂直翻转

        // correct_function(fixPosX, fixPosY);     // 矫正偏差

        // // flip_vertical(fixPosY);                 // 垂直翻转

        // // flip_vertical(Car.carPosition.y);

        // // 赋值
        // Car.carPositionFixed.x = fixPosX;
        // Car.carPositionFixed.y = fixPosY;

        correct_function_2(Car.carPosition, Car.carPositionFixed, 0);
        
    }
}

void Monitoring::run(std::vector<Yolo::Detection>& rtxCars, cv::Mat& img, std::vector<car>& result) {
    // 
    allCar.clear();
    allCar.shrink_to_fit();
    allArmor.clear();
    allArmor.shrink_to_fit();

    // 分析信息 (中途已透视变换)
    analyseData(rtxCars, allCar, allArmor, img, warpmatrix);

    // 矫正透视变换点
    fixCarPosition(allCar);

    // 结果赋值
    result = this->allCar;
}

#endif  // _MONITORING_HPP_