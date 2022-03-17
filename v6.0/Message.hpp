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

    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    cv::Point2f blue1_2;
    cv::Point2f blue2_2;
    cv::Point2f red1_2;
    cv::Point2f red2_2;

} CarInfoSend;


class Message {
    private:
        CarInfoSend     PC_1;
        // uint            gray_num;
        int             sameColorNumFrames = 0;         // 车同号同色出现的帧数记数, 用于判断 交换颜色模式 0/1
        int             singleFrameSameColorNumSroce;   // 单帧出现同车同号的得分，出现一次+1分，出现两次则可以确认并且给 sameColorNumFrames += 1

    public:
        Message();
        ~Message();
        void init();
        CarInfoSend operator()(std::vector<car>& result, CarInfoSend& PC_2, bool& receive);
};

Message::Message() {
    init();
}

Message::~Message() {

}

void Message::init() {
    Message::singleFrameSameColorNumSroce = 0;

    PC_1.gray_num = 0;
    PC_1.swapColorModes = 0;

    PC_1.blue1.x = -1;
    PC_1.blue1.y = -1;

    PC_1.blue2.x = -1;
    PC_1.blue2.y = -1;

    PC_1.red1.x = -1;
    PC_1.red1.y = -1;

    PC_1.red2.x = -1;
    PC_1.red2.y = -1;

    PC_1.blue1_2.x = -1;
    PC_1.blue1_2.y = -1;

    PC_1.blue2_2.x = -1;
    PC_1.blue2_2.y = -1;

    PC_1.red1_2.x = -1;
    PC_1.red1_2.y = -1;

    PC_1.red2_2.x = -1;
    PC_1.red2_2.y = -1;
}

float getDistance(const cv::Point2f& point1, const cv::Point2f& point2) {
    static double distance;
    distance = sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
    return distance;
}

cv::Point2f getMean(const cv::Point2f& point1, const cv::Point2f& point2) {
    static cv::Point2f mean;
    // 加个判断? 当双点差距过大, return point1;
    mean = cv::Point2f((point1.x+point2.x)/2.0, (point1.y+point2.y)/2.0);
    return mean;
}

// 当出现同号同色的两车时才调用该函数
/*需要 Point1 的点为靠近哨岗的点
    判断 y轴值相近 asb<=20 (以主哨岗为原点, 808边为y轴)
        选择面积小的点为point1
*/
void swapPointCheck(cv::Point2f& point1, cv::Point2f& point2) {
    if ( std::fabs(point1.y-point2.y) <= 20 && 
         ((point1.x*point1.y) > (point2.x*point2.y)) 
    ) {
        std::swap(point1, point2);
    }
}

/*
    传入的参数表示的 [同号同色的车] 在主[1]副[2]哨岗上检测到的 [不同位置]
    主哨岗: CarLocation1, CarLocation1_2
    副哨岗: CarLocation2, CarLocation2_2
*/
void CarPlaceMerge(cv::Point2f& CarLocation1, cv::Point2f& CarLocation1_2, cv::Point2f& CarLocation2, cv::Point2f& CarLocation2_2) {
    // 0: 无任何数据
    if ( (CarLocation1   == cv::Point2f(-1,-1) && CarLocation2   == cv::Point2f(-1,-1)) && 
         (CarLocation1_2 == cv::Point2f(-1,-1) && CarLocation2_2 == cv::Point2f(-1,-1))
    ) {
        return ;
    }
    // 1: 主哨岗检测到一个位置, 其他为空
    else if ( (CarLocation1   != cv::Point2f(-1,-1) && CarLocation2   == cv::Point2f(-1,-1)) &&
              (CarLocation1_2 == cv::Point2f(-1,-1) && CarLocation2_2 == cv::Point2f(-1,-1))
    ) {
        // CarLocation1 无需改动
        return ;
    }
    // 1: 副哨岗检测到一个位置, 其他为空
    else if ( (CarLocation1   == cv::Point2f(-1,-1) && CarLocation2   != cv::Point2f(-1,-1)) &&
              (CarLocation1_2 == cv::Point2f(-1,-1) && CarLocation2_2 == cv::Point2f(-1,-1))
    ) {
        CarLocation1 = CarLocation2;
    }
    // 2: 主副哨岗各检测到一个位置
    else if ( (CarLocation1   != cv::Point2f(-1,-1) && CarLocation2   != cv::Point2f(-1,-1)) &&
              (CarLocation1_2 == cv::Point2f(-1,-1) && CarLocation2_2 == cv::Point2f(-1,-1))
    ) {
        float distanceCar = getDistance(CarLocation1, CarLocation2);
        // 两点距离超过 [50] 厘米, 判定为两台不一样的车
        if (distanceCar > 50) {
            // CarLocation1 无需改动
            CarLocation1_2 = CarLocation2;  // 副哨岗赋值给主哨岗
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
        // 判定为同一台车的坐标，数据求mean融合
        else {
            CarLocation1 = getMean(CarLocation1, CarLocation2);
        }
    }
    // 2: 主哨岗检测到两个位置，副哨岗无
    else if ( (CarLocation1   != cv::Point2f(-1,-1) && CarLocation2   == cv::Point2f(-1,-1)) &&
              (CarLocation1_2 != cv::Point2f(-1,-1) && CarLocation2_2 == cv::Point2f(-1,-1))
    ) {
        // 无需操作
    }
    // 2: 副哨岗检测到两个位置，主哨岗无
    else if ( (CarLocation1   == cv::Point2f(-1,-1) && CarLocation2   != cv::Point2f(-1,-1)) &&
              (CarLocation1_2 == cv::Point2f(-1,-1) && CarLocation2_2 != cv::Point2f(-1,-1))
    ) {
        CarLocation1    = CarLocation2;
        CarLocation1_2  = CarLocation2_2;
    }
    // 3: 主哨岗检测到两个位置, 副哨岗检测到一个位置
    else if ( (CarLocation1   != cv::Point2f(-1,-1) && CarLocation2   != cv::Point2f(-1,-1)) &&
              (CarLocation1_2 != cv::Point2f(-1,-1) && CarLocation2_2 == cv::Point2f(-1,-1))
    ) {
        // 分别求 CarLocation1到CarLocation2的距离 和 CarLocation1_2到CarLocation2的距离
        float distanceCar_1 = getDistance(CarLocation1,   CarLocation2);
        float distanceCar_2 = getDistance(CarLocation1_2, CarLocation2);
        // 若CarLocation1比CarLocation1_2 离 CarLocation2 更近,则判断CarLocation1和CarLocation2判断的为同一台车
        if (distanceCar_1 < distanceCar_2) {
            CarLocation1 = getMean(CarLocation1, CarLocation2);
            // CarLocation1_2 无需改动
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
        // 反之
        else {
            // CarLocation1 无需改动
            CarLocation1_2 = getMean(CarLocation1_2, CarLocation2);
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
    }
    // 3: 主哨岗检测到一个位置, 副哨岗检测到两个位置
    else if ( (CarLocation1   != cv::Point2f(-1,-1) && CarLocation2   != cv::Point2f(-1,-1)) &&
              (CarLocation1_2 == cv::Point2f(-1,-1) && CarLocation2_2 != cv::Point2f(-1,-1))
    ) { 
        // 分别求 CarLocation1到CarLocation2的距离 和 CarLocation1到CarLocation2_2的距离
        float distanceCar_1 = getDistance(CarLocation1, CarLocation2);
        float distanceCar_2 = getDistance(CarLocation1, CarLocation2_2);
        // 若CarLocation2比CarLocation2_2 离 CarLocation1 更近,则判断CarLocation2和CarLocation1判断的为同一台车
        if (distanceCar_1 < distanceCar_2) {
            CarLocation1    = getMean(CarLocation1, CarLocation2);
            CarLocation1_2  = CarLocation2_2;   // 副哨岗赋值给主哨岗
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
        // 反之
        else {
            CarLocation1    = getMean(CarLocation1, CarLocation2_2);
            CarLocation1_2  = CarLocation2;
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
    }
    // 4: 主副哨岗都检测到两个位置
    else if ( (CarLocation1   != cv::Point2f(-1,-1) && CarLocation2   != cv::Point2f(-1,-1)) &&
              (CarLocation1_2 != cv::Point2f(-1,-1) && CarLocation2_2 != cv::Point2f(-1,-1))
    ) {
        // 相信单哨岗的 y轴从小到大排序放置 和 位置交换?
        float distanceCar_1  = getDistance(CarLocation1, CarLocation2);
        float distanceCar_2  = getDistance(CarLocation1, CarLocation2_2);
        if (distanceCar_1 < distanceCar_2) {
            CarLocation1    = getMean(CarLocation1,   CarLocation2);
            CarLocation1_2  = getMean(CarLocation1_2, CarLocation2_2);
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
        // 反之
        else {
            CarLocation1    = getMean(CarLocation1,   CarLocation2_2);
            CarLocation1_2  = getMean(CarLocation1_2, CarLocation2);
            swapPointCheck(CarLocation1, CarLocation1_2);
        }
    }
}

//自定义排序函数  
bool SortByCarPositionFixedY(const car& car1, const car& car2)   //注意：本函数的参数的类型一定要与vector中元素的类型一致  
{   
    return car1.carPositionFixed.y < car2.carPositionFixed.y;  //升序排列
}

CarInfoSend Message::operator()(std::vector<car>& result, CarInfoSend& PC_2, bool& receive) {
    init();
//     cv::Point2f carPositionFixed;   // 矫正后的`世界地图`上的坐标点
//     int         color;              // 0蓝 1红 2黑
//     int         num;                // 1 / 2
    
    // 对 std::vector<car> result 以 y轴从小到大排序
    std::sort(result.begin(), result.end(), SortByCarPositionFixedY);

    for (car& info: result) {
        if (info.num == 1) {            // 先车牌 <-- `1`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                if (PC_1.blue1 == cv::Point2f(-1 ,-1)) {    // 若未出现该 类型[color, num] 的车，则直接赋值
                    PC_1.blue1 = info.carPositionFixed;
                }
                else {                                      // 若已经出现该 类型[color, num] 的车，则为blue1_2赋值
                    PC_1.blue1_2 = info.carPositionFixed;
                    swapPointCheck(PC_1.blue1, PC_1.blue1_2);
                }
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                if (PC_1.red1 == cv::Point2f(-1 ,-1)) {
                    PC_1.red1 = info.carPositionFixed;
                }
                else {
                    PC_1.red1_2 = info.carPositionFixed;
                    swapPointCheck(PC_1.red1, PC_1.red1_2);
                }  
            }
            else if (info.color == 2) { // 后颜色 <-- `gray`2
                PC_1.gray_num += 1;
            }
        }
        else if (info.num == 2) {       // 先车牌 <-- `2`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                if (PC_1.blue2 == cv::Point2f(-1 ,-1)) {
                    PC_1.blue2 = info.carPositionFixed;
                }
                else {
                    PC_1.blue2_2 = info.carPositionFixed;
                    swapPointCheck(PC_1.blue2, PC_1.blue2_2);
                }
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                if (PC_1.red2 == cv::Point2f(-1 ,-1)) {
                    PC_1.red2 = info.carPositionFixed;
                }
                else {
                    PC_1.red2_2 = info.carPositionFixed;
                    swapPointCheck(PC_1.red2, PC_1.red2_2);
                }
            }
            else if (info.color == 2) { // 后颜色 <-- `gray`2
                PC_1.gray_num += 1;
            }
        }
    }
    

    /* 数据融合
        bool receive 表示是否接收到副哨岗的信息
    */
    if (receive) {
        // PC_1.blue1   PC_1.blue1_2    PC_2.blue1  PC_1.blue1_2  ==> 除去 cv::Point2f(-1, -1)的, 剩下的去聚类, 以此类推
        CarPlaceMerge(PC_1.blue1, PC_1.blue1_2, PC_2.blue1, PC_1.blue1_2);
        CarPlaceMerge(PC_1.red1,  PC_1.red1_2,  PC_2.red1,  PC_1.red1_2);
        CarPlaceMerge(PC_1.blue2, PC_1.blue2_2, PC_2.blue2, PC_1.blue2_2);
        CarPlaceMerge(PC_1.red2,  PC_1.red2_2,  PC_2.red2,  PC_1.red2_2);

        // 选两个哨岗检测出的死车max数
        PC_1.gray_num = std::max(PC_1.gray_num, PC_2.gray_num);
    }

    /*  有同号同色情况, Message::singleFrameSameColorNumSroce += 1
     */
    if (PC_1.blue1   != cv::Point2f(-1, -1) &&
        PC_1.blue1_2 != cv::Point2f(-1, -1)) {
        Message::singleFrameSameColorNumSroce += 1;
    }
    if (PC_1.blue2   != cv::Point2f(-1, -1) &&
        PC_1.blue2_2 != cv::Point2f(-1, -1)) {
        Message::singleFrameSameColorNumSroce += 1;
    }
    if (PC_1.red1    != cv::Point2f(-1, -1) &&
        PC_1.red1_2  != cv::Point2f(-1, -1)) {
        Message::singleFrameSameColorNumSroce += 1;
    }
    if (PC_1.red2    != cv::Point2f(-1, -1) &&
        PC_1.red2_2  != cv::Point2f(-1, -1)) {
        Message::singleFrameSameColorNumSroce += 1;
    }

    // 记录同车同号出现的帧数
    /*
        一: 无车死亡
            [无灰车 PC_1.gray_num == 0, 单帧 车同号同色 情况出现两次 Message::singleFrameSameColorNumSroce == 2 ] -------> sameColorNumFrames+=1
        二: 己方未死, 敌方死一台
            [有灰车 PC_1.gray_num == 1, 单帧 车同号同色 情况出现一次 Message::singleFrameSameColorNumSroce == 1 ] -------> sameColorNumFrames+=1
        三: 己方死一台, 敌方未死
            [有灰车 PC_1.gray_num == 1, 单帧 车同号同色 情况出现一次 Message::singleFrameSameColorNumSroce == 1 ] -------> sameColorNumFrames+=1
        四: 双方各死一台
            因为 PC_1.gray_num == 2, !故而不能判断正确结果

        但由于三四都为己方死一台车为前提, 就不必考虑是否误伤队友, 这时 sameColorNumFrames和socketInfo.swapColorModes 的值是什么已经无所谓了
    */
    if (Message::singleFrameSameColorNumSroce + PC_1.gray_num == 2) {
        sameColorNumFrames += 1;            // 车同号同号色出现的帧数+1
    }

    // 当sameColorNumFrames累积达到某个阈值时，即多次检测到[两辆同车同号]的情况出现时 --> 确认潜伏模式后的车车颜色交换情况
    if (Message::sameColorNumFrames >= 30) {
        PC_1.swapColorModes = 1;      // 达到阈值, 判定 1 --> 最麻烦的情况, 敌我车辆同号同色
    }

    /*  这里添加坐标轴转换代码

    */

    return this->PC_1;
}

#endif  // _MESSAGE_HPP_