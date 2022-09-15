#ifndef SENTRY_MODULE_WATCHTOWRE_WATCHTOWRE_HPP_
#define SENTRY_MODULE_WATCHTOWRE_WATCHTOWRE_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "position.hpp"
// #include "msgs.hpp" // 声明结构体 ReceiveOtherWatchtowerInfo ReceiveCarInfo

// 哨岗的信息
struct WatchtowerInfo {
    int gray_num;           // 当前帧灰车的数量

    int swap_color_mode;    // 交换颜色模式: 交换后异号异色车--0  交换后同号同色车--1

    // 在潜伏模式后, 卧底车的颜色会变成敌方的颜色
    // 在潜伏模式后, 己方车辆发送 [自身的地图坐标&队友是否死亡的信息] 给主哨岗, 主哨岗取其信息进行卧底车牌号判断
    int discoloration_num;  // [卧底]变色车牌号
    
    //判断 灰车是否在buff区F6和F1中
    bool gray_on_buff_F6;
    bool gray_on_buff_F1;

    // 四类车坐标
    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    // 潜伏模式后, 可能会出现同色同号的车, 建立temp用于存储其坐标
    cv::Point2f blue1_temp;
    cv::Point2f blue2_temp;
    cv::Point2f red1_temp;
    cv::Point2f red2_temp;

    // 灰车坐标
    cv::Point2f gray1;
    cv::Point2f gray2;
    cv::Point2f gray3;
    cv::Point2f gray4;

    // 空构造
    WatchtowerInfo() {
        this->gray_num = 0;

        this->swap_color_mode = 0;

        this->discoloration_num = -1;

        this->gray_on_buff_F6 = false;
        this->gray_on_buff_F1 = false;

        this->blue1.x = -1; this->blue1.y = -1;
        this->blue2.x = -1; this->blue2.y = -1;
        this->red1.x = -1;  this->red1.y = -1;
        this->red2.x = -1;  this->red2.y = -1;

        this->blue1_temp.x = -1; this->blue1_temp.y = -1;
        this->blue2_temp.x = -1; this->blue2_temp.y = -1;
        this->red1_temp.x = -1;  this->red1_temp.y = -1;
        this->red2_temp.x = -1;  this->red2_temp.y = -1;

        this->gray1.x = -1; this->gray1.y = -1;
        this->gray2.x = -1; this->gray2.y = -1;
        this->gray3.x = -1; this->gray3.y = -1;
        this->gray4.x = -1; this->gray4.y = -1;
    }

    void init() {
        this->gray_num = 0;
        this->swap_color_mode = 0;
        this->discoloration_num = -1;
        this->gray_on_buff_F6 = false;
        this->gray_on_buff_F1 = false;
        this->blue1.x = -1; this->blue1.y = -1;
        this->blue2.x = -1; this->blue2.y = -1;
        this->red1.x = -1;  this->red1.y = -1;
        this->red2.x = -1;  this->red2.y = -1;
        this->blue1_temp.x = -1; this->blue1_temp.y = -1;
        this->blue2_temp.x = -1; this->blue2_temp.y = -1;
        this->red1_temp.x = -1;  this->red1_temp.y = -1;
        this->red2_temp.x = -1;  this->red2_temp.y = -1;
        this->gray1.x = -1; this->gray1.y = -1;
        this->gray2.x = -1; this->gray2.y = -1;
        this->gray3.x = -1; this->gray3.y = -1;
        this->gray4.x = -1; this->gray4.y = -1;
    }
};

// 车发送给哨岗的信息
struct CarSendWatchtower {
    cv::Point2f position;           // Car当前坐标
    bool        teammates_alive;    // 队友是否存活  false / true

    CarSendWatchtower() : position(cv::Point2f(-1,-1)), teammates_alive(false) {}
};

struct ReceiveCarInfo {
    CarSendWatchtower robo_car_info;
    bool              comfirm_receipt;          // 本回合 是否 接收成功

    ReceiveCarInfo() {
        robo_car_info   = CarSendWatchtower();
        comfirm_receipt = false;
    }
};

struct ReceiveOtherWatchtowerInfo {
    WatchtowerInfo  other_tower_info;
    bool            comfirm_receipt;            // 本回合 是否 接收成功
    bool            other_tower_online;         // 副哨岗 是否 在线

    ReceiveOtherWatchtowerInfo() {
        other_tower_info    = WatchtowerInfo();
        comfirm_receipt     = false;
        other_tower_online  = false;
    }
};

struct ReceiveInfo {
    ReceiveOtherWatchtowerInfo  receive_tower_info;
    ReceiveCarInfo              receive_car_1_info;
    ReceiveCarInfo              receive_car_2_info;

    ReceiveInfo() {
        receive_tower_info = ReceiveOtherWatchtowerInfo();
        receive_car_1_info = ReceiveCarInfo();
        receive_car_2_info = ReceiveCarInfo();
    }
};

struct WatchtowerConfig {
    // 
    int watchtoer_identity;

    // 
    float merge_same_car_distance;
    
    // 
    int judge_same_color_num_mode_frames_thershold;
    int judge_same_color_num_mode_conditions;

    //
    int judge_discoloration_car_num_frames_thershold;
    int team_color;
    float judge_discoloration_car_num_distance;

    //
    float judge_emeny_car_distance;

    // 空构造
    WatchtowerConfig() {
        this->watchtoer_identity = 0;
        this->merge_same_car_distance = 0;
        this->judge_same_color_num_mode_frames_thershold = 0;
        this->judge_same_color_num_mode_conditions = 0;
        this->judge_discoloration_car_num_frames_thershold = 0;
        this->team_color = 0;
        this->judge_discoloration_car_num_distance = 0;
        this->judge_emeny_car_distance = 0;
    }

    WatchtowerConfig(int wi_, float mscd_, int jscnmft_, int jscnmc_, int jdcnft_, int tc_, float jdcnd_, float jecd_) {
        this->watchtoer_identity = wi_;
        this->merge_same_car_distance = mscd_;
        this->judge_same_color_num_mode_frames_thershold = jscnmft_;
        this->judge_same_color_num_mode_conditions = jscnmc_;
        this->judge_discoloration_car_num_frames_thershold = jdcnft_;
        this->team_color = tc_;
        this->judge_discoloration_car_num_distance = jdcnd_;
        this->judge_emeny_car_distance = jecd_;
    }
};


class Watchtower {
    private:
        WatchtowerInfo      tower_info_;
        WatchtowerConfig    tower_config_;

    private:
        // 副哨岗仅需下两函数(carsInfo2WatchtowerInfo 和 checkBuffStatus)
        void        carsInfo2WatchtowerInfo(std::vector<CarsInfo> &cars_info, WatchtowerInfo &tower_info);
        void        checkBuffStatus(WatchtowerInfo &tower_info);
        
        // 辅助函数
        float       getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2);
        int         relu(const int &frame);
        
        // 双哨岗数据融合
        cv::Point2f chooseOne(const cv::Point2f &position_1, const cv::Point2f &position_2);
        void        mergeBuffStatus(bool &tower_gray_on_buff, bool &other_tower_gray_on_buff);
        void        mergeGrayCarInfo(int &tower_gray_num, int &other_tower_gray_num);
        void        mergeSingleClassCarPositionInfo(cv::Point2f &position_1, cv::Point2f &position_1_temp, cv::Point2f &position_2, cv::Point2f &position_2_temp);  // 对于单个类别的四个信息进行融合
        void        mergeWatchtowerInfo(WatchtowerInfo &tower_info, ReceiveOtherWatchtowerInfo &receive_tower_info);

        // 调查潜伏变色模式
        int         voteSameColorNumMode(WatchtowerInfo &tower_info);
        void        judgeSwapColorMode(WatchtowerInfo &tower_info, bool other_tower_online, bool comfirm_receipt_tower);

        // 同色同号下, 调查己方变色车辆号数
        void        voteDiscolorationCarNum(WatchtowerInfo &tower_info, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info, int vote[3]);   // #include <unordered_map>  unordered_map<string, int> vote; vote["car_1"] = 0; vote["car_2"] = 0;
        void        judgeDiscolorationCarNum(WatchtowerInfo &tower_info, bool other_tower_online, bool comfirm_receipt_tower, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info);
        
        // 同色同号下, [过滤掉己方坐标信息, 只保留敌方坐标信息]; 需要借助Car1和Car2发来的坐标信息
        void        filterCarPositionProcess(cv::Point2f &position_1, cv::Point2f &position_2, const cv::Point2f &car_position);
        void        filterOwnCarPosition(WatchtowerInfo &tower_info, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info);

    public:
        Watchtower(WatchtowerConfig &tower_config);
        ~Watchtower();

        // 副哨岗用
        WatchtowerInfo get_watchtower_info(std::vector<CarsInfo> &cars_info);  

        // 主哨岗用
        WatchtowerInfo get_watchtower_info(std::vector<CarsInfo> &cars_info, ReceiveOtherWatchtowerInfo &receive_tower_info, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info);
};


#endif // SENTRY_MODULE_WATCHTOWRE_WATCHTOWRE_HPP_