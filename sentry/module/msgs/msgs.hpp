#ifndef SENTRY_MODULE_MSGS_MSGS_HPP_
#define SENTRY_MODULE_MSGS_MSGS_HPP_

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <zmq.hpp>
#include "watchtower.hpp" // 声明结构体 WatchtowerInfo

// // 车发送给哨岗的信息
// struct CarSendWatchtower {
//     cv::Point2f position;           // Car当前坐标
//     bool        teammates_alive;    // 队友是否存活  false / true
// };

// struct ReceiveCarInfo {
//     CarSendWatchtower robo_car_info;
//     bool              comfirm_receipt;          // 本回合 是否 接收成功
// };

// struct ReceiveOtherWatchtowerInfo {
//     WatchtowerInfo  other_tower_info;
//     bool            comfirm_receipt;            // 本回合 是否 接收成功
//     bool            other_tower_online;         // 副哨岗 是否 在线
// };

// struct ReceiveInfo {
//     ReceiveOtherWatchtowerInfo  receive_tower_info;
//     ReceiveCarInfo              receive_car_1_info;
//     ReceiveCarInfo              receive_car_2_info;
// };

// 发送给 RoboCar 的信息
struct SendToCarInfo {
    // int gray_num;           // 当前帧灰车的数量

    int swap_color_mode;    // 交换颜色模式: 交换后异号异色车--0  交换后同号同色车--1

    int discoloration_num;  // 卧底

    //判断 灰车是否在buff区F6和F1中
    bool gray_on_buff_F6;
    bool gray_on_buff_F1;

    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    SendToCarInfo() {
        this->swap_color_mode = 0;
        this->discoloration_num = -1;
        this->blue1 = cv::Point2f(-1, -1);
        this->blue2 = cv::Point2f(-1, -1);
        this->red1  = cv::Point2f(-1, -1);
        this->red2  = cv::Point2f(-1, -1);
        this->gray_on_buff_F6 = false;
        this->gray_on_buff_F1 = false;
    }
    SendToCarInfo(int &scm_, int &dn_, cv::Point2f &b1_, cv::Point2f &b2_, cv::Point2f &r1_, cv::Point2f &r2_, bool &gobF6_, bool &gobF1_) {
        this->swap_color_mode = scm_;
        this->discoloration_num = dn_;
        this->blue1 = b1_;
        this->blue2 = b2_;
        this->red1  = r1_;
        this->red2  = r2_;
        this->gray_on_buff_F6 = gobF6_;
        this->gray_on_buff_F1 = gobF1_;
    }
};

struct MsgsConfig {
    int         watchtoer_identity; // 0主 / 1副

    std::string tower_self_ip;

    std::string other_tower_ip;
    std::string car_1_ip;
    std::string car_2_ip;

    MsgsConfig() : watchtoer_identity(-1), tower_self_ip(""), other_tower_ip(""), car_1_ip(""), car_2_ip("") {}
    MsgsConfig(int wd_, std::string tsi_, std::string oti_, std::string c1i_, std::string c2i_) : watchtoer_identity(wd_), tower_self_ip(tsi_), other_tower_ip(oti_), car_1_ip(c1i_), car_2_ip(c2i_) {}
};

class Msgs {
private:
    MsgsConfig      msgs_config_;

    ReceiveInfo     receive_info_;
    
    SendToCarInfo   send_to_car_info_;

    zmq::socket_t   publisher_send_info;

    zmq::socket_t   subscriber_receive_other_tower;
    zmq::socket_t   subscriber_receive_car_1;
    zmq::socket_t   subscriber_receive_car_2;

private:
    ReceiveCarInfo              receiveCarInfo(ReceiveCarInfo &receive_car_info, zmq::socket_t &subscriber_receive_car);
    ReceiveOtherWatchtowerInfo  receiveOtherWatchtowerInfo(ReceiveOtherWatchtowerInfo &receive_tower_info, zmq::socket_t &subscriber_receive_other_tower);
    ReceiveInfo                 receiveInfo(ReceiveInfo &receive_info_, zmq::socket_t &subscriber_receive_other_tower, zmq::socket_t &subscriber_receive_car_1, zmq::socket_t &subscriber_receive_car_2);

public:
    Msgs(MsgsConfig msgs_config);
    ~Msgs();
    
    ReceiveInfo                 get_receive_info();
    void                        send_info(WatchtowerInfo &tower_info, zmq::socket_t &publisher_send_info);
};

#endif // SENTRY_MODULE_MSGS_MSGS_HPP_