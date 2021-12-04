#ifndef _MESSAGE_HPP_
#define _MESSAGE_HPP_

#include <iostream>
#include <vector>
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
    // int colorType;
    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    cv::Point2f gray1;
    cv::Point2f gray2;
    cv::Point2f gray3;
    cv::Point2f gray4;
} CarPositionSend;


class Message {
    private:
        CarPositionSend socketInfo;
        uint            gray_num;
    public:
        Message();
        ~Message();
        void init();
        CarPositionSend operator()(std::vector<car>& result);
};

Message::Message() {
    init();
}

Message::~Message() {

}

void Message::init() {
    gray_num = 1;

    socketInfo.blue1.x = -1;
    socketInfo.blue1.y = -1;

    socketInfo.blue2.x = -1;
    socketInfo.blue2.y = -1;

    socketInfo.red1.x = -1;
    socketInfo.red1.y = -1;

    socketInfo.red2.x = -1;
    socketInfo.red2.y = -1;

    socketInfo.gray1.x = -1;
    socketInfo.gray1.y = -1;

    socketInfo.gray2.x = -1;
    socketInfo.gray2.y = -1;

    socketInfo.gray3.x = -1;
    socketInfo.gray3.y = -1;

    socketInfo.gray4.x = -1;
    socketInfo.gray4.y = -1;
}


CarPositionSend Message::operator()(std::vector<car>& result) {
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
                socketInfo.blue1.x = info.carPositionFixed.x;
                socketInfo.blue1.y = info.carPositionFixed.y;
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                socketInfo.red1.x = info.carPositionFixed.x;
                socketInfo.red1.y = info.carPositionFixed.y;
            }
            else if (info.color == 2) { // 后颜色 <-- `gray`2
                switch (gray_num)
                {
                    case 1:
                        socketInfo.gray1.x = info.carPositionFixed.x;
                        socketInfo.gray1.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    case 2:
                        socketInfo.gray2.x = info.carPositionFixed.x;
                        socketInfo.gray2.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    case 3:
                        socketInfo.gray3.x = info.carPositionFixed.x;
                        socketInfo.gray3.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    case 4:
                        socketInfo.gray4.x = info.carPositionFixed.x;
                        socketInfo.gray4.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    default:
                        break;
                }
            }
        }
        else if (info.num == 2) {       // 先车牌 <-- `2`
            if (info.color == 0) {      // 后颜色 <-- `blue`0
                socketInfo.blue2.x = info.carPositionFixed.x;
                socketInfo.blue2.y = info.carPositionFixed.y;
            }
            else if (info.color == 1) { // 后颜色 <-- `red` 1
                socketInfo.red2.x = info.carPositionFixed.x;
                socketInfo.red2.y = info.carPositionFixed.y;
            }
            else if (info.color == 2) { // 后颜色 <-- `gray`2
                switch (gray_num)
                {
                    case 1:
                        socketInfo.gray1.x = info.carPositionFixed.x;
                        socketInfo.gray1.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    case 2:
                        socketInfo.gray2.x = info.carPositionFixed.x;
                        socketInfo.gray2.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    case 3:
                        socketInfo.gray3.x = info.carPositionFixed.x;
                        socketInfo.gray3.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    case 4:
                        socketInfo.gray4.x = info.carPositionFixed.x;
                        socketInfo.gray4.y = info.carPositionFixed.y;
                        gray_num++;
                        break;
                    default:
                        break;
                }
            }
        }
    }

    return this->socketInfo;
}


#endif  // _MESSAGE_HPP_