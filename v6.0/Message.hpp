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

    // int colorType;
    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    cv::Point2f blue1_2;
    cv::Point2f blue2_2;
    cv::Point2f red1_2;
    cv::Point2f red2_2;

    // cv::Point2f gray1;
    // cv::Point2f gray2;
    // cv::Point2f gray3;
    // cv::Point2f gray4;
} CarInfoSend;


class Message {
    private:
        CarInfoSend     socketInfo;
        // uint            gray_num;
        int             sameColorNumFrames = 0;         // 同车同号出现的帧数记数, 用于判断 交换颜色模式 0/1
        int             singleFrameSameColorNumSroce;   // 单帧出现同车同号的得分，出现一次+1分，出现两次则可以确认并且给 sameColorNumFrames += 1
    public:
        Message();
        ~Message();
        void init();
        CarInfoSend operator()(std::vector<car>& result);
};

Message::Message() {
    init();
}

Message::~Message() {

}

void Message::init() {
    Message::singleFrameSameColorNumSroce = 0;

    socketInfo.gray_num = 0;

    socketInfo.blue1.x = -1;
    socketInfo.blue1.y = -1;

    socketInfo.blue2.x = -1;
    socketInfo.blue2.y = -1;

    socketInfo.red1.x = -1;
    socketInfo.red1.y = -1;

    socketInfo.red2.x = -1;
    socketInfo.red2.y = -1;

    socketInfo.blue1_2.x = -1;
    socketInfo.blue1_2.y = -1;

    socketInfo.blue2_2.x = -1;
    socketInfo.blue2_2.y = -1;

    socketInfo.red1_2.x = -1;
    socketInfo.red1_2.y = -1;

    socketInfo.red2_2.x = -1;
    socketInfo.red2_2.y = -1;

    // socketInfo.gray1.x = -1;
    // socketInfo.gray1.y = -1;

    // socketInfo.gray2.x = -1;
    // socketInfo.gray2.y = -1;

    // socketInfo.gray3.x = -1;
    // socketInfo.gray3.y = -1;

    // socketInfo.gray4.x = -1;
    // socketInfo.gray4.y = -1;
}


CarInfoSend Message::operator()(std::vector<car>& result) {
    init();
//     cv::Point2f carPositionFixed;   // 矫正后的`世界地图`上的坐标点
//     int         color;              // 0蓝 1红 2黑
//     int         num;                // 1 / 2
    for (car& info: result) {
        /*
        
            这里添加坐标轴转换代码

        */
        if (info.num == 1) {            // 先车牌 <-- `1`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                if (socketInfo.blue1.x == -1 && socketInfo.blue1.y == -1) {    // 若未出现该 类型[color, num] 的车，则直接赋值
                    socketInfo.blue1.x = info.carPositionFixed.x;
                    socketInfo.blue1.y = info.carPositionFixed.y;
                }
                else {                                                          // 若已经出现该 类型[color, num] 的车，则为blue1_2赋值
                    socketInfo.blue1_2.x = info.carPositionFixed.x;
                    socketInfo.blue1_2.y = info.carPositionFixed.y;
                    Message::singleFrameSameColorNumSroce += 1;
                }
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                if (socketInfo.red1.x == -1 && socketInfo.red1.y == -1) {
                    socketInfo.red1.x = info.carPositionFixed.x;
                    socketInfo.red1.y = info.carPositionFixed.y;
                }
                else {
                    socketInfo.red1_2.x = info.carPositionFixed.x;
                    socketInfo.red1_2.y = info.carPositionFixed.y;
                    Message::singleFrameSameColorNumSroce += 1;
                }
            }
            else if (info.color == 2) { // 后颜色 <-- `gray`2
                socketInfo.gray_num += 1;
            //     switch (socketInfo.gray_num)
            //     {
            //         case 1:
            //             socketInfo.gray1.x = info.carPositionFixed.x;
            //             socketInfo.gray1.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         case 2:
            //             socketInfo.gray2.x = info.carPositionFixed.x;
            //             socketInfo.gray2.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         case 3:
            //             socketInfo.gray3.x = info.carPositionFixed.x;
            //             socketInfo.gray3.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         case 4:
            //             socketInfo.gray4.x = info.carPositionFixed.x;
            //             socketInfo.gray4.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         default:
            //             break;
            //     }
            }
        }
        else if (info.num == 2) {       // 先车牌 <-- `2`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                if (socketInfo.blue2.x == -1 && socketInfo.blue2.y == -1) {
                    socketInfo.blue2.x = info.carPositionFixed.x;
                    socketInfo.blue2.y = info.carPositionFixed.y;
                }
                else {
                    socketInfo.blue2_2.x = info.carPositionFixed.x;
                    socketInfo.blue2_2.y = info.carPositionFixed.y;
                    Message::singleFrameSameColorNumSroce += 1;
                }
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                if (socketInfo.red2.x == -1 && socketInfo.red2.y == -1) {
                    socketInfo.red2.x = info.carPositionFixed.x;
                    socketInfo.red2.y = info.carPositionFixed.y;
                }
                else {
                    socketInfo.red2_2.x = info.carPositionFixed.x;
                    socketInfo.red2_2.y = info.carPositionFixed.y;
                    Message::singleFrameSameColorNumSroce += 1;
                }
            }
            else if (info.color == 2) { // 后颜色 <-- `gray`2
                socketInfo.gray_num += 1;
            //     switch (socketInfo.gray_num)
            //     {
            //         case 1:
            //             socketInfo.gray1.x = info.carPositionFixed.x;
            //             socketInfo.gray1.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         case 2:
            //             socketInfo.gray2.x = info.carPositionFixed.x;
            //             socketInfo.gray2.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         case 3:
            //             socketInfo.gray3.x = info.carPositionFixed.x;
            //             socketInfo.gray3.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         case 4:
            //             socketInfo.gray4.x = info.carPositionFixed.x;
            //             socketInfo.gray4.y = info.carPositionFixed.y;
            //             socketInfo.gray_num++;
            //             break;
            //         default:
            //             break;
            //     }
            }
        }
    }


/* 数据融合



*/

    // 记录同车同号出现的帧数
    /*
        一: 无车死亡
            [无灰车 socketInfo.gray_num == 0, 单帧同车同色情况出现两次 Message::singleFrameSameColorNumSroce == 2 ] -------> sameColorNumFrames+=1
        二: 己方未死, 敌方死一台
            [有灰车 socketInfo.gray_num == 1, 单帧同车同色情况出现一次 Message::singleFrameSameColorNumSroce == 1 ] -------> sameColorNumFrames+=1
        三: 己方死一台, 敌方未死
            [有灰车 socketInfo.gray_num == 1, 单帧同车同色情况出现一次 Message::singleFrameSameColorNumSroce == 1 ] -------> sameColorNumFrames+=1
        四: 双方各死一台
            因为 socketInfo.gray_num == 2, !故而不能判断正确结果

        但由于三四都为己方死一台车为前提, 就不必考虑是否误伤队友, 这时 sameColorNumFrames和socketInfo.swapColorModes 的值是什么已经无所谓了
    */
    if (Message::singleFrameSameColorNumSroce + socketInfo.gray_num == 2) {
        sameColorNumFrames += 1;
    }
    // 当sameColorNumFrames累积达到某个阈值时，即多次检测到[两辆同车同号]的情况出现时 --> 确认潜伏模式后的车车颜色交换情况
    if (sameColorNumFrames >= 30) {
        socketInfo.swapColorModes = 1;
    }

    return this->socketInfo;
}


#endif  // _MESSAGE_HPP_