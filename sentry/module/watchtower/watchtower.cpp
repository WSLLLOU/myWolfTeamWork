#include "watchtower.hpp"

float Watchtower::getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2) {
    static double distance;
    distance = sqrtf(powf((point_1.x - point_2.x),2) + powf((point_1.y - point_2.y),2));
    return distance;
}

int Watchtower::relu(const int &frame) {
    static int zero = 0;
    // return std::max(frame, zero);
    // return frame > zero ? frame : 0;
    if (frame < zero) {
        return zero;
    }
    return frame;
}

void Watchtower::carsInfo2WatchtowerInfo(std::vector<CarsInfo> &cars_info, WatchtowerInfo &tower_info) {
    tower_info.init();

    // // 意义不明
    // std::sort(cars_info.begin(), cars_info.end(), [&](const CarsInfo &car_1, const CarsInfo &car_2){
    //     return car_1.position.y < car_2.position.y;  //升序排列
    // });

//     cv::Point2f position;    // 矫正后的`世界地图`上的坐标点
//     int         color;       // 0蓝 1红 2黑
//     int         num;         // 1 / 2
    for (CarsInfo &car : cars_info) {
        if (car.num == 1) {            // 先判断车牌 <-- `1`
            if (car.color == 0) {      // 后判断颜色 <-- `blue` 0
                if (tower_info.blue1 == cv::Point2f(-1 ,-1)) {    // 若未出现该 类型[color, num] 的车，则直接赋值
                    tower_info.blue1 = car.position;
                }
                else {                                      // 若已经出现该 类型[color, num] 的车，则为blue1_2赋值
                    tower_info.blue1_temp = car.position;
                }
            }
            else if (car.color == 1) { // 后判断颜色 <-- `red` 1
                if (tower_info.red1 == cv::Point2f(-1 ,-1)) {
                    tower_info.red1 = car.position;
                }
                else {
                    tower_info.red1_temp = car.position;
                }  
            }
        }
        else if (car.num == 2) {       // 先判断车牌 <-- `2`
            if (car.color == 0) {      // 后判断颜色 <-- `blue` 0
                if (tower_info.blue2 == cv::Point2f(-1 ,-1)) {
                    tower_info.blue2 = car.position;
                }
                else {
                    tower_info.blue2_temp = car.position;
                }
            }
            else if (car.color == 1) { // 后颜色 <-- `red` 1
                if (tower_info.red2 == cv::Point2f(-1 ,-1)) {
                    tower_info.red2 = car.position;
                }
                else {
                    tower_info.red2_temp = car.position;
                }
            }
        }
        else {  // 现在只剩下 [灰装甲板](num==-1) / [无装甲板] 车辆了,需要把 灰车数据保存下来
            if (car.color == 2) {
                tower_info.gray_num += 1;

                if (tower_info.gray_num == 1) {
                    tower_info.gray1 = car.position;
                }
                else if (tower_info.gray_num == 2) {
                    tower_info.gray2 = car.position;
                }
                else if (tower_info.gray_num == 3) {
                    tower_info.gray3 = car.position;
                }
                else if (tower_info.gray_num == 4) {
                    tower_info.gray4 = car.position;
                }
            }
        }
    }
}


void Watchtower::checkBuffStatus(WatchtowerInfo &tower_info) {
    // 判断 F6
    // 世界地图上的 F6 buff area (适当扩大)
    static cv::Rect F6 = cv::Rect(255-40,23 , 48+60,54+30); // x, y, width, height
    if (tower_info.gray_on_buff_F6 == false && 
        tower_info.gray1 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F6 = F6.contains(tower_info.gray1);
    }
    if (tower_info.gray_on_buff_F6 == false && 
        tower_info.gray2 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F6 = F6.contains(tower_info.gray2);
    }
    if (tower_info.gray_on_buff_F6 == false && 
        tower_info.gray3 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F6 = F6.contains(tower_info.gray3);
    }
    if (tower_info.gray_on_buff_F6 == false && 
        tower_info.gray4 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F6 = F6.contains(tower_info.gray4);
    }

    // 判断 F1
    // 世界地图上的 F1 buff area (适当扩大)
    static cv::Rect F1 = cv::Rect(145-20,731-20 , 48+30,54+30); // x, y, width, height
    if (tower_info.gray_on_buff_F1 == false && 
        tower_info.gray1 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F1 = F1.contains(tower_info.gray1);
    }
    if (tower_info.gray_on_buff_F1 == false && 
        tower_info.gray2 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F1 = F1.contains(tower_info.gray2);
    }
    if (tower_info.gray_on_buff_F1 == false && 
        tower_info.gray3 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F1 = F1.contains(tower_info.gray3);
    }
    if (tower_info.gray_on_buff_F1 == false && 
        tower_info.gray4 != cv::Point2f(-1,-1)) {
                tower_info.gray_on_buff_F1 = F1.contains(tower_info.gray4);
    }
}

// chooseOne 原本为 getMean 函数, 两方哨岗得出的数据距离真实值差的不多, 但是偏差的方位相反, 使用求均获得真实值在理论上确实挺好;
// 但由于副哨岗和主哨岗之间帧数差距过大, 会造成定位数据的急剧抖动, 故而弃掉, 只要确保主副哨岗都检测到该类车辆的时候, 只返回主哨岗检测的坐标数据就好
cv::Point2f Watchtower::chooseOne(const cv::Point2f &position_1, const cv::Point2f &position_2) {
    // static cv::Point2f mean;
    // // TODO: 加个判断 当双点差距过大, return position_1;
    // mean = cv::Point2f((position_1.x+position_2.x)/2.0, (position_1.y+position_2.y)/2.0);
    // return mean;

    return position_1;
}


void Watchtower::mergeBuffStatus(bool &tower_gray_on_buff, bool &other_tower_gray_on_buff) {
    tower_gray_on_buff = tower_gray_on_buff || other_tower_gray_on_buff;
}

// 灰车坐标用于判断两个 buff 的占位情况, 融合 buff-status 和 gray-num 就行, 不用融合灰车坐标了
void Watchtower::mergeGrayCarInfo(int &tower_gray_num, int &other_tower_gray_num) {
    tower_gray_num = tower_gray_num > other_tower_gray_num ? tower_gray_num : other_tower_gray_num;
}


// 对于单个类别的四个信息进行融合
void Watchtower::mergeSingleClassCarPositionInfo(cv::Point2f &position_1, cv::Point2f &position_1_temp, cv::Point2f &position_2, cv::Point2f &position_2_temp){
    // 0: 无任何数据
    if ( (position_1      == cv::Point2f(-1,-1) && position_2      == cv::Point2f(-1,-1)) && 
         (position_1_temp == cv::Point2f(-1,-1) && position_2_temp == cv::Point2f(-1,-1))
    ) {
        return ;
    }
    // 1: 主哨岗检测到一个位置, 其他为空
    else if ( (position_1      != cv::Point2f(-1,-1) && position_2      == cv::Point2f(-1,-1)) &&
              (position_1_temp == cv::Point2f(-1,-1) && position_2_temp == cv::Point2f(-1,-1))
    ) {
        // position_1 无需改动
        return ;
    }
    // 1: 副哨岗检测到一个位置, 其他为空
    else if ( (position_1      == cv::Point2f(-1,-1) && position_2      != cv::Point2f(-1,-1)) &&
              (position_1_temp == cv::Point2f(-1,-1) && position_2_temp == cv::Point2f(-1,-1))
    ) {
        position_1 = position_2;
    }
    // 2: 主副哨岗各检测到一个位置
    else if ( (position_1      != cv::Point2f(-1,-1) && position_2      != cv::Point2f(-1,-1)) &&
              (position_1_temp == cv::Point2f(-1,-1) && position_2_temp == cv::Point2f(-1,-1))
    ) {
        float car_distance = getDistance(position_1, position_2);
        // 两点距离超过 [60] 厘米, 判定为两台不一样的车
        if (car_distance > this->tower_config_.merge_same_car_distance) {
            // position_1 无需改动
            position_1_temp = position_2;  // 副哨岗赋值给主哨岗
        }
        // 判定为同一台车的坐标，数据求mean融合
        else {
            position_1 = chooseOne(position_1, position_2);
        }
    }
    // 2: 主哨岗检测到两个位置，副哨岗无
    else if ( (position_1      != cv::Point2f(-1,-1) && position_2      == cv::Point2f(-1,-1)) &&
              (position_1_temp != cv::Point2f(-1,-1) && position_2_temp == cv::Point2f(-1,-1))
    ) {
        // 无需操作
    }
    // 2: 副哨岗检测到两个位置，主哨岗无
    else if ( (position_1      == cv::Point2f(-1,-1) && position_2      != cv::Point2f(-1,-1)) &&
              (position_1_temp == cv::Point2f(-1,-1) && position_2_temp != cv::Point2f(-1,-1))
    ) {
        position_1      = position_2;
        position_1_temp = position_2_temp;
    }
    // 3: 主哨岗检测到两个位置, 副哨岗检测到一个位置
    else if ( (position_1      != cv::Point2f(-1,-1) && position_2      != cv::Point2f(-1,-1)) &&
              (position_1_temp != cv::Point2f(-1,-1) && position_2_temp == cv::Point2f(-1,-1))
    ) {
        float car_distance_1 = getDistance(position_1,      position_2);
        float car_distance_2 = getDistance(position_1_temp, position_2);
        // 若position_1比position_1_temp 离 position_2 更近,则判断position_1和position_2判断的为同一台车
        if (car_distance_1 < car_distance_2) {
            position_1 = chooseOne(position_1, position_2);
            // position_2 无需改动
        }
        // 反之
        else {
            // position_1 无需改动
            position_1_temp = chooseOne(position_1_temp, position_2);
        }
    }
    // 3: 主哨岗检测到一个位置, 副哨岗检测到两个位置
    else if ( (position_1      != cv::Point2f(-1,-1) && position_2   != cv::Point2f(-1,-1)) &&
              (position_1_temp == cv::Point2f(-1,-1) && position_2_temp != cv::Point2f(-1,-1))
    ) { 
        // 分别求 position_1到position_2的距离 和 position_1到position_2_temp的距离
        float car_distance_1 = getDistance(position_1, position_2);
        float car_distance_2 = getDistance(position_1, position_2_temp);
        // 若position_2比position_2_temp 离 position_1 更近,则判断position_2和position_1判断的为同一台车
        if (car_distance_1 < car_distance_2) {
            position_1    = chooseOne(position_1, position_2);
            position_1_temp  = position_2_temp;   // 副哨岗赋值给主哨岗
        }
        // 反之
        else {
            position_1    = chooseOne(position_1, position_2_temp);
            position_1_temp  = position_2;
        }
    }
    // 4: 主副哨岗都检测到两个位置
    else if ( (position_1   != cv::Point2f(-1,-1) && position_2   != cv::Point2f(-1,-1)) &&
              (position_1_temp != cv::Point2f(-1,-1) && position_2_temp != cv::Point2f(-1,-1))
    ) {
        // 相信单哨岗的 y轴从小到大排序放置 和 位置交换?
        float car_distance_1  = getDistance(position_1, position_2);
        float car_distance_2  = getDistance(position_1, position_2_temp);
        if (car_distance_1 < car_distance_2) {
            position_1    = chooseOne(position_1,   position_2);
            position_1_temp  = chooseOne(position_1_temp, position_2_temp);
        }
        // 反之
        else {
            position_1    = chooseOne(position_1,   position_2_temp);
            position_1_temp  = chooseOne(position_1_temp, position_2);
        }
    }
}

// 融合两个哨岗的信息
void Watchtower::mergeWatchtowerInfo(WatchtowerInfo &tower_info, ReceiveOtherWatchtowerInfo &receive_tower_info) {
    if (receive_tower_info.other_tower_online && receive_tower_info.comfirm_receipt) {

        WatchtowerInfo other_tower_info = receive_tower_info.other_tower_info;

        mergeSingleClassCarPositionInfo(tower_info.blue1, tower_info.blue1_temp, other_tower_info.blue1, other_tower_info.blue1_temp);
        mergeSingleClassCarPositionInfo(tower_info.red1,  tower_info.red1_temp,  other_tower_info.red1,  other_tower_info.red1_temp);
        mergeSingleClassCarPositionInfo(tower_info.blue2, tower_info.blue2_temp, other_tower_info.blue2, other_tower_info.blue2_temp);
        mergeSingleClassCarPositionInfo(tower_info.red2,  tower_info.red2_temp,  other_tower_info.red2,  other_tower_info.red2_temp);

        mergeBuffStatus(tower_info.gray_on_buff_F6, other_tower_info.gray_on_buff_F6);
        mergeBuffStatus(tower_info.gray_on_buff_F1, other_tower_info.gray_on_buff_F1);

        mergeGrayCarInfo(tower_info.gray_num, other_tower_info.gray_num);

    }
}


int Watchtower::voteSameColorNumMode(WatchtowerInfo &tower_info) {
    int vote_single_frame_same_color_num_mode = 0;
    
    /*同色同号检测
        有同号同色情况, vote_single_frame_same_color_num_mode += 1
    */
    if (tower_info.blue1      != cv::Point2f(-1, -1) &&
        tower_info.blue1_temp != cv::Point2f(-1, -1)) {
        vote_single_frame_same_color_num_mode += 1;
    }
    if (tower_info.blue2      != cv::Point2f(-1, -1) &&
        tower_info.blue2_temp != cv::Point2f(-1, -1)) {
        vote_single_frame_same_color_num_mode += 1;
    }
    if (tower_info.red1       != cv::Point2f(-1, -1) &&
        tower_info.red1_temp  != cv::Point2f(-1, -1)) {
        vote_single_frame_same_color_num_mode += 1;
    }
    if (tower_info.red2       != cv::Point2f(-1, -1) &&
        tower_info.red2_temp  != cv::Point2f(-1, -1)) {
        vote_single_frame_same_color_num_mode += 1;
    }

    return vote_single_frame_same_color_num_mode;
}


void Watchtower::judgeSwapColorMode(WatchtowerInfo &tower_info, bool other_tower_online, bool comfirm_receipt_tower) {
    static int same_color_num_mode_frames = 0;

    static int frames_thershold      = this->tower_config_.judge_same_color_num_mode_frames_thershold;
    static int judge_mode_conditions = this->tower_config_.judge_same_color_num_mode_conditions;
    
    if (same_color_num_mode_frames < frames_thershold) {
        
        // comfirm_receipt_tower 表示是否接收到 哨岗消息    接收到true 未接收到false
        // other_tower_online    表示副哨岗是否在线         在线true   不在线false
        // 若 (!other_tower_online)==false, 则 comfirm_receipt_tower ==true  ==> 副哨岗在线,  且接收到副哨岗消息,  此时进行数据融合后的      [同色同号]&[卧底] 检测
        // 若 (!other_tower_online)==ture,  则 comfirm_receipt_tower ==false ==> 副哨岗掉线,  必收不到副哨岗消息,  此时进行用主哨岗信息的     [同色同号]&[卧底] 检测
        // 若 (!other_tower_online)==false, 则 comfirm_receipt_tower ==false ==> 副哨岗在线,  但未接收到副哨岗消息, 未有数据融合,此时不进行(因缺失完整信息) [同色同号]&[卧底] 检测
        if (!(other_tower_online) || comfirm_receipt_tower) {
            int vote = 0;
            vote = voteSameColorNumMode(tower_info);
        
            //
            if (judge_mode_conditions == 0) { // "strict"
                // 记录同车同号出现的帧数
                /*
                    一: 无车死亡
                        [无灰车 tower_info.gray_num == 0, 单帧 车同号同色 情况出现两次 vote == 2 ] -------> same_color_num_mode_frames+=1
                    二: 己方未死, 敌方死一台
                        [有灰车 tower_info.gray_num == 1, 单帧 车同号同色 情况出现一次 vote == 1 ] -------> same_color_num_mode_frames+=1
                    三: 己方死一台, 敌方未死
                        [有灰车 tower_info.gray_num == 1, 单帧 车同号同色 情况出现一次 vote == 1 ] -------> same_color_num_mode_frames+=1
                    四: 双方各死一台
                        [tower_info.gray_num == 2,       单帧 车同号同色 情况出现一次 vote == 1 ] -------> same_color_num_mode_frames+=1

                    但由于三四都为己方死一台车为前提, 此时不必考虑是否会误伤队友, 则无须判断潜伏后的交换颜色情况了
                */
                if (tower_info.gray_num != 2 && vote + tower_info.gray_num == 2) {
                    same_color_num_mode_frames += 1;            // 车同号同号色出现的帧数+1
                }
                else if (tower_info.gray_num == 2 && vote==1) {
                    same_color_num_mode_frames += 1;
                }
                else {
                    same_color_num_mode_frames -= 2;
                    same_color_num_mode_frames = relu(same_color_num_mode_frames);
                }
            } else if (judge_mode_conditions == 1) { // "relaxed"
                // 宽松的条件, 不限场上多少辆车, 只要主哨岗连续紧密地检测到同色同号条件即可
                if (vote == 1) {
                    same_color_num_mode_frames += 1;       // 车同号同号色出现的帧数+1
                }
                else {
                    same_color_num_mode_frames -= 2;
                    same_color_num_mode_frames = relu(same_color_num_mode_frames);
                }
            }
        }
    } else if (same_color_num_mode_frames >= frames_thershold){
        tower_info.swap_color_mode = 1;
    }
}

void Watchtower::voteDiscolorationCarNum(WatchtowerInfo &tower_info, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info, int vote[3]) {
    static int          team_color          = this->tower_config_.team_color;
    static float        distance_threshold  = this->tower_config_.judge_discoloration_car_num_distance;

    
    auto get_min_car_distance = [&](cv::Point2f position_1, cv::Point2f position_2, cv::Point2f compare_position) -> float {
        static cv::Point2f  nothing     = cv::Point2f(-1,-1);
        float               min_car_dis = getDistance(cv::Point2f(808+10, 448+10), nothing);
        float               car_dis     = 0;

        if (position_1 != nothing) {
            car_dis = getDistance(position_1, compare_position);
            if (car_dis < min_car_dis) {
                min_car_dis = car_dis;
            }
        }
        if (position_2 != nothing) {
            car_dis = getDistance(position_1, compare_position);
            if (car_dis < min_car_dis) {
                min_car_dis = car_dis;
            }
        }

        return min_car_dis;
    };


    if (receive_car_1_info.comfirm_receipt == true) {
        float min_car_distance = 0;

        if (team_color == 0) {      // "blue"
            min_car_distance = get_min_car_distance(tower_info.red1,  tower_info.red1_temp,  receive_car_1_info.robo_car_info.position);
        } else if (team_color == 1) {  // "red"
            min_car_distance = get_min_car_distance(tower_info.blue1, tower_info.blue1_temp, receive_car_1_info.robo_car_info.position);
        }

        if (min_car_distance <= distance_threshold) {
            vote[1] = 1;
        }
        if (vote[1] == 0 && receive_car_1_info.robo_car_info.teammates_alive == false) {
            vote[2] = 1;
        }
    }

    if (receive_car_2_info.comfirm_receipt == true) {
        float min_car_distance = 0;

        if (team_color == 0) {      // "blue"
            min_car_distance = get_min_car_distance(tower_info.red2,  tower_info.red2_temp,  receive_car_2_info.robo_car_info.position);
        } else if (team_color == 1) {  // "red"
            min_car_distance = get_min_car_distance(tower_info.blue2, tower_info.blue2_temp, receive_car_2_info.robo_car_info.position);
        }

        if (min_car_distance <= distance_threshold) {
            vote[2] = 1;
        }
        if (vote[2] == 0 && receive_car_2_info.robo_car_info.teammates_alive == false) {
            vote[1] = 1;
        }
    }
}

void Watchtower::judgeDiscolorationCarNum(WatchtowerInfo &tower_info, bool other_tower_online, bool comfirm_receipt_tower, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info) {
    if (tower_info.swap_color_mode != 1) {
        return ;
    }

    static int  discoloration_car_1_num_frames = 0;
    static int  discoloration_car_2_num_frames = 0;

    static int  frames_thershold = this->tower_config_.judge_discoloration_car_num_frames_thershold;

    if (discoloration_car_1_num_frames < frames_thershold &&
        discoloration_car_2_num_frames < frames_thershold ) {
            // comfirm_receipt_tower 表示是否接收到 哨岗消息    接收到true 未接收到false
            // other_tower_online    表示副哨岗是否在线         在线true   不在线false
            // 若 (!other_tower_online)==false, 则 comfirm_receipt_tower ==true  ==> 副哨岗在线,  且接收到副哨岗消息,  此时进行数据融合后的      [同色同号]&[卧底] 检测
            // 若 (!other_tower_online)==ture,  则 comfirm_receipt_tower ==false ==> 副哨岗掉线,  必收不到副哨岗消息,  此时进行用主哨岗信息的     [同色同号]&[卧底] 检测
            // 若 (!other_tower_online)==false, 则 comfirm_receipt_tower ==false ==> 副哨岗在线,  但未接收到副哨岗消息, 未有数据融合,此时不进行(因缺失完整信息) [同色同号]&[卧底] 检测
            if (!(other_tower_online) || comfirm_receipt_tower) {
                int vote[3] = {0, 0 ,0};
                voteDiscolorationCarNum(tower_info, receive_car_1_info, receive_car_2_info, vote);

                if (vote[1] == 1)       { discoloration_car_1_num_frames += 1; discoloration_car_1_num_frames = relu(discoloration_car_1_num_frames); }
                else if (vote[1] == 0)  { discoloration_car_1_num_frames -= 2; discoloration_car_1_num_frames = relu(discoloration_car_1_num_frames); }

                if (vote[2] == 1)       { discoloration_car_2_num_frames += 1; discoloration_car_2_num_frames = relu(discoloration_car_2_num_frames); }
                else if (vote[2] == 0)  { discoloration_car_2_num_frames -= 2; discoloration_car_2_num_frames = relu(discoloration_car_2_num_frames); }
            }
    } else if (discoloration_car_1_num_frames >= frames_thershold ) {
        tower_info.discoloration_num = 1;   // 1号是变色车辆
    } else if (discoloration_car_2_num_frames >= frames_thershold) {
        tower_info.discoloration_num = 2;   // 2号是变色车辆
    }
}

// 当出现同号同色的两车时才调用该函数
/*
    // 4个类别 blue1, blue2, red1, red2
    position_1 和 position_2 为同一个类别
        position_1为当前类别检测到的第一个坐标
        position_2为当前类别检测到的第二个坐标
      当position_2有数据时, position_1必有数据
    
      car_position 为当前 1号车/2号车 的坐标位置

    最终处理后 position_1 用于存储敌方坐标信息, 若无法判断敌方坐标, 则p1、p2的坐标信息都改为 (-1, -1) 
*/
void Watchtower::filterCarPositionProcess(cv::Point2f &position_1, cv::Point2f &position_2, const cv::Point2f &car_position) {
    static cv::Point2f nothing = cv::Point2f(-1,-1);
    
    float position_1_to_car_position_distance = 0;  // cm
    float position_2_to_car_position_distance = 0;  // cm

    // 若 position_1 不为空数据, position_1 与 car_position 进行距离计算
    if (position_1 != nothing) {
        position_1_to_car_position_distance = getDistance(position_1, car_position);
    }
    if (position_2 != nothing) {
        position_2_to_car_position_distance = getDistance(position_2, car_position);
    }
    
    static float distance_threshold = this->tower_config_.judge_emeny_car_distance; // cm

    // position_2 离 car_position 更远
    if (position_1_to_car_position_distance < position_2_to_car_position_distance) {
        // 若该 position_2 与 car_position 的距离 >= 阈值, 判断 position_2 为敌方坐标
        if (position_2_to_car_position_distance >= distance_threshold) { 
            std::swap(position_1, position_2);
        }
        // 两个距离都未超过阈值, 三个坐标靠的很近, 无须哨岗提供敌方坐标信息
        else {
            position_1.x=-1,position_1.y=-1;
            position_2.x=-1,position_2.y=-1;
        }
    }
    // position_1 和 position_2  距离 car_position 同样远
    else if (position_1_to_car_position_distance == position_2_to_car_position_distance) {
        // 可能 position_1 和 position_2 都没有数据, 无须操作
        // 可能 position_1 和 position_2 距离 car_position 同样远, 很刁钻, 没什么办法
    }
    // 可能只检测到了 position_1 , 或者刚好 position_1 为敌方坐标
    else if (position_1_to_car_position_distance > position_2_to_car_position_distance) {
        // 若该 position_1 与 car_position 的距离 >= 阈值, 判断 position_1 为敌方坐标
        if (position_1_to_car_position_distance >= distance_threshold) {
            // 无须操作
        }
        // 两个距离都未超过阈值, 
        //   或许三个坐标靠的很近, 无须哨岗提供敌方坐标信息
        //   又或许只检测到自方车辆, 也无须哨岗提供敌方坐标信息
        else {
            position_1.x=-1,position_1.y=-1;
            position_2.x=-1,position_2.y=-1;
        }
    }
}


void Watchtower::filterOwnCarPosition(WatchtowerInfo &tower_info, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info) {
    if (tower_info.swap_color_mode != 1) {
        return ;
    }

    if (receive_car_1_info.comfirm_receipt == true) {
        filterCarPositionProcess(tower_info.blue1, tower_info.blue1_temp, receive_car_1_info.robo_car_info.position);
        filterCarPositionProcess(tower_info.red1,  tower_info.red1_temp,  receive_car_1_info.robo_car_info.position);
    } else {
        // 所有有关于1车牌号的数据置空
        tower_info.blue1.x      = -1;   tower_info.blue1.y      = -1;
        tower_info.blue1_temp.x = -1;   tower_info.blue1_temp.y = -1;
        tower_info.red1.x       = -1;   tower_info.red1.y       = -1;
        tower_info.red1_temp.x  = -1;   tower_info.red1_temp.y  = -1;
    }

    if (receive_car_1_info.comfirm_receipt == true) {
        filterCarPositionProcess(tower_info.blue2, tower_info.blue2_temp, receive_car_2_info.robo_car_info.position);
        filterCarPositionProcess(tower_info.red2,  tower_info.red2_temp,  receive_car_2_info.robo_car_info.position);
    } else {
        // 所有有关于1车牌号的数据置空
        tower_info.blue2.x      = -1;   tower_info.blue2.y      = -1;
        tower_info.blue2_temp.x = -1;   tower_info.blue2_temp.y = -1;
        tower_info.red2.x       = -1;   tower_info.red2.y       = -1;
        tower_info.red2_temp.x  = -1;   tower_info.red2_temp.y  = -1;
    }
}


// 副哨岗接口
WatchtowerInfo Watchtower::get_watchtower_info(std::vector<CarsInfo> &cars_info) {
    carsInfo2WatchtowerInfo(cars_info, this->tower_info_);
    checkBuffStatus(this->tower_info_);

    return this->tower_info_;
}

// 主哨岗接口
WatchtowerInfo Watchtower::get_watchtower_info(std::vector<CarsInfo> &cars_info, ReceiveOtherWatchtowerInfo &receive_tower_info, ReceiveCarInfo &receive_car_1_info, ReceiveCarInfo &receive_car_2_info) {
    carsInfo2WatchtowerInfo(cars_info, this->tower_info_);
    checkBuffStatus(this->tower_info_);

    mergeWatchtowerInfo(this->tower_info_, receive_tower_info);

    judgeSwapColorMode(this->tower_info_, receive_tower_info.other_tower_online, receive_tower_info.comfirm_receipt);

    judgeDiscolorationCarNum(this->tower_info_, receive_tower_info.other_tower_online, receive_tower_info.comfirm_receipt, receive_car_1_info, receive_car_2_info);

    filterOwnCarPosition(this->tower_info_, receive_car_1_info, receive_car_2_info);

    return this->tower_info_;
}

Watchtower::Watchtower(WatchtowerConfig &tower_config) {
    this->tower_config_ = tower_config;
    this->tower_info_.init();
}

Watchtower::~Watchtower() {

}