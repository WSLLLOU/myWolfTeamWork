#ifndef _MESSAGE_HPP_
#define _MESSAGE_HPP_

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <Correct.hpp>
#include <algorithm>

// typedef struct
// {
//     cv::Rect    img_r;              // `图像`上的矩形框信息
//     cv::Point2f carPosition;        // 矫正前的`世界地图`上的坐标点
//     cv::Point2f carPositionFixed;   // 矫正后的`世界地图`上的坐标点
//     int         color;              // 0蓝 1红 2黑
//     int         num;                // 1 / 2
// } car;

typedef struct {			// 套接字内容

    int gray_num;           // 当前帧灰车的数量

    int swapColorModes;     // 交换颜色模式: 交换后异号异色车--0  交换后同号同色车--1

    // 在潜伏模式后, 穿山甲的颜色会变成敌方的颜色
    // 在潜伏模式后, 己方车辆发送 [自身的地图坐标&队友是否死亡的信息] 给主哨岗, 主哨岗取其信息进行卧底车牌号判断
    int pangolin;           // 穿山甲
    
    // 占着茅坑不拉屎 1 2
    bool a_dog_in_the_toilet_on_shit_1;
    bool a_dog_in_the_toilet_on_shit_2;

    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    cv::Point2f blue1_2;
    cv::Point2f blue2_2;
    cv::Point2f red1_2;
    cv::Point2f red2_2;

    cv::Point2f gray_1;
    cv::Point2f gray_2;
    cv::Point2f gray_3;
    cv::Point2f gray_4;

} CarInfoSend;

class Message {
    private:
        CarInfoSend     PC_2;

        const std::string whoAmI    = WHO_AM_I; // blue or red 现在是蓝方还是红方
        // // // 检测同色同号主要靠主哨岗检测，检测同色同号条件为 [严格] or [宽松]
        // // //      [严格]需要场上四台车都在, 但条件还是很严格(主哨岗连续检测+条件严格), 主哨岗检测不完全就不行;
        // // //      [宽松]不限场上多少台车, 主哨岗能连续检测到一对同色同号即可, 但主哨岗检测不到还是不行;
        // // //const std::string swapColorCondition = "relaxed"; // strict or relaxed

    public:
        Message();
        ~Message();
        void init();

        CarInfoSend operator()(std::vector<car>& result);

        void swapPointCheck(cv::Point2f& point1, cv::Point2f& point2);
};

Message::Message() {
    init();
}

Message::~Message() {

}

void Message::init() {
    PC_2.pangolin = -1;  // 穿山甲-1初始化

    PC_2.a_dog_in_the_toilet_on_shit_1 = false;
    PC_2.a_dog_in_the_toilet_on_shit_2 = false;

    PC_2.gray_num = 0;
    PC_2.swapColorModes = 0;

    PC_2.blue1.x = -1;
    PC_2.blue1.y = -1;

    PC_2.blue2.x = -1;
    PC_2.blue2.y = -1;

    PC_2.red1.x = -1;
    PC_2.red1.y = -1;

    PC_2.red2.x = -1;
    PC_2.red2.y = -1;

    PC_2.blue1_2.x = -1;
    PC_2.blue1_2.y = -1;

    PC_2.blue2_2.x = -1;
    PC_2.blue2_2.y = -1;

    PC_2.red1_2.x = -1;
    PC_2.red1_2.y = -1;

    PC_2.red2_2.x = -1;
    PC_2.red2_2.y = -1;

    PC_2.gray_1.x = -1;
    PC_2.gray_1.y = -1;

    PC_2.gray_2.x = -1;
    PC_2.gray_2.y = -1;

    PC_2.gray_3.x = -1;
    PC_2.gray_3.y = -1;

    PC_2.gray_4.x = -1;
    PC_2.gray_4.y = -1;
}

float getDistance(const cv::Point2f& point1, const cv::Point2f& point2) {
    static double distance;
    distance = sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
    return distance;
}
// 当出现同号同色的两车时才调用该函数
/*      需要 Point1 的点为靠近哨岗的点
        判断 y轴值相近 fasb<=20 (以主哨岗为参考点测距)
        选择面积小的点为point1
*/
void Message::swapPointCheck(cv::Point2f& point1, cv::Point2f& point2) {
    static cv::Point2f myPosition;
    if (Message::whoAmI == "blue") {
        myPosition = cv::Point2f(0.0, 808.0);  // 当前地图中主哨岗的全局坐标
    }
    else if (Message::whoAmI == "red") {
        myPosition = cv::Point2f(0.0, 0.0);    // 当前地图中主哨岗的全局坐标
    }

    // static cv::Point2f myPosition = cv::Point2f(0.0, 0.0);  // 当前哨岗全局坐标, 若换坐标系表示的时候，需要改

    if ( ( std::fabs(point1.y-point2.y) <= 20 ) &&          // 808的边用y表示, 若这两个坐标的y轴相近,则需要取判断那个比较靠近主哨岗, !如果要更换坐标系表示,y不再表示808这条边,则这段代码需要改动为x
         ( getDistance(point1, myPosition) > getDistance(point2, myPosition) )
    ) {
        std::swap(point1, point2);
    }
}


//自定义排序函数  
bool SortByCarPositionFixedY(const car& _car1_sort, const car& _car2_sort)   //注意：本函数的参数的类型一定要与vector中元素的类型一致  
{   
    return _car1_sort.carPositionFixed.y < _car2_sort.carPositionFixed.y;  //升序排列
}

CarInfoSend Message::operator()(std::vector<car>& result) {
    init();
//     cv::Point2f carPositionFixed;   // 矫正后的`世界地图`上的坐标点
//     int         color;              // 0蓝 1红 2黑
//     int         num;                // 1 / 2
    
    // 对 std::vector<car> result 以 y轴从小到大排序
    std::sort(result.begin(), result.end(), SortByCarPositionFixedY);

    for (car& info: result) {
        if (info.num == 1) {            // 先车牌 <-- `1`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                if (PC_2.blue1 == cv::Point2f(-1 ,-1)) {    // 若未出现该 类型[color, num] 的车，则直接赋值
                    PC_2.blue1 = info.carPositionFixed;
                }
                else {                                      // 若已经出现该 类型[color, num] 的车，则为blue1_2赋值
                    PC_2.blue1_2 = info.carPositionFixed;
                    swapPointCheck(PC_2.blue1, PC_2.blue1_2);
                }
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                if (PC_2.red1 == cv::Point2f(-1 ,-1)) {
                    PC_2.red1 = info.carPositionFixed;
                }
                else {
                    PC_2.red1_2 = info.carPositionFixed;
                    swapPointCheck(PC_2.red1, PC_2.red1_2);
                }  
            }
        }
        else if (info.num == 2) {       // 先车牌 <-- `2`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                if (PC_2.blue2 == cv::Point2f(-1 ,-1)) {
                    PC_2.blue2 = info.carPositionFixed;
                }
                else {
                    PC_2.blue2_2 = info.carPositionFixed;
                    swapPointCheck(PC_2.blue2, PC_2.blue2_2);
                }
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                if (PC_2.red2 == cv::Point2f(-1 ,-1)) {
                    PC_2.red2 = info.carPositionFixed;
                }
                else {
                    PC_2.red2_2 = info.carPositionFixed;
                    swapPointCheck(PC_2.red2, PC_2.red2_2);
                }
            }
        }
        else {  // 现在只剩下 [灰装甲板](num==-1) / [无装甲板] 车辆了,需要把 灰车数据保存下来
            if (info.color == 2) {
                PC_2.gray_num += 1;

                if (PC_2.gray_num == 1) {
                    PC_2.gray_1 = info.carPositionFixed;
                }
                else if (PC_2.gray_num == 2) {
                    PC_2.gray_2 = info.carPositionFixed;
                }
                else if (PC_2.gray_num == 3) {
                    PC_2.gray_3 = info.carPositionFixed;
                }
                else if (PC_2.gray_num == 4) {
                    PC_2.gray_4 = info.carPositionFixed;
                }
            }
        }
    }
    
    // 占着茅坑不拉屎判断
    // PC_2.a_dog_in_the_toilet_on_shit_1 一号茅坑判断
    // static cv::Rect buff_1 当前地图的 [一号茅坑]的(左上角位置和宽高大小)
    static cv::Rect buff_1 = cv::Rect(255,23 , 48,54); // x, y, width, height
    if (PC_2.a_dog_in_the_toilet_on_shit_1 == false && 
        PC_2.gray_1 != cv::Point2f(-1,-1)) {
            PC_2.a_dog_in_the_toilet_on_shit_1 = buff_1.contains(PC_2.gray_1);
    }
    else if (PC_2.a_dog_in_the_toilet_on_shit_1 == false && 
        PC_2.gray_2 != cv::Point2f(-1,-1)) {
                PC_2.a_dog_in_the_toilet_on_shit_1 = buff_1.contains(PC_2.gray_2);
    }
    else if (PC_2.a_dog_in_the_toilet_on_shit_1 == false && 
        PC_2.gray_3 != cv::Point2f(-1,-1)) {
                PC_2.a_dog_in_the_toilet_on_shit_1 = buff_1.contains(PC_2.gray_3);
    }
    else if (PC_2.a_dog_in_the_toilet_on_shit_1 == false && 
        PC_2.gray_4 != cv::Point2f(-1,-1)) {
                PC_2.a_dog_in_the_toilet_on_shit_1 = buff_1.contains(PC_2.gray_4);
    }
    // PC_2.a_dog_in_the_toilet_on_shit_2 二号茅坑判断
    // static cv::Rect buff_2 当前地图的 [二号茅坑]的(左上角位置和宽高大小)
    static cv::Rect buff_2 = cv::Rect(145,731 , 48,54); // x, y, width, height
    if (PC_2.a_dog_in_the_toilet_on_shit_2 == false && 
        PC_2.gray_1 != cv::Point2f(-1,-1)) {
            PC_2.a_dog_in_the_toilet_on_shit_2 = buff_2.contains(PC_2.gray_1);
    }
    else if (PC_2.a_dog_in_the_toilet_on_shit_2 == false && 
        PC_2.gray_2 != cv::Point2f(-1,-1)) {
                PC_2.a_dog_in_the_toilet_on_shit_2 = buff_2.contains(PC_2.gray_2);
    }
    else if (PC_2.a_dog_in_the_toilet_on_shit_2 == false && 
        PC_2.gray_3 != cv::Point2f(-1,-1)) {
                PC_2.a_dog_in_the_toilet_on_shit_2 = buff_2.contains(PC_2.gray_3);
    }
    else if (PC_2.a_dog_in_the_toilet_on_shit_2 == false && 
        PC_2.gray_4 != cv::Point2f(-1,-1)) {
                PC_2.a_dog_in_the_toilet_on_shit_2 = buff_2.contains(PC_2.gray_4);
    }

    
    return this->PC_2;
}

#endif  // _MESSAGE_HPP_