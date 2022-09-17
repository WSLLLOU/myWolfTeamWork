#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include "get_warp_matrix.hpp"
#include "yolov5.hpp"
#include "position.hpp"
#include "watchtower.hpp"
#include "msgs.hpp"
#include "visualize.hpp"
#include <zmq.hpp>
#include <thread>
#include <mutex>

// get config
cv::FileStorage fs_read("../configs/config.yaml", cv::FileStorage::READ);

int         watchtower_identity;
int         team_color;
int         fix_position_method;
int         click_point_method;
float       watchtower_high;
float       half_car_high;
float       offset_x;
float       offset_y;
float       merge_same_car_distance;
int         judge_same_color_num_mode_frames_thershold;
int         judge_same_color_num_mode_conditions;
int         judge_discoloration_car_num_frames_thershold;
int         judge_discoloration_car_num_distance;
float       judge_emeny_car_distance;
std::string tower_self_ip;
std::string other_tower_ip;
std::string car_1_ip;
std::string car_2_ip;

ReceiveInfo receive_info;

Visualize   visualize;

std::mutex  mtx;

void receive(Msgs &msgs) {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 休眠20ms
mtx.lock();
        receive_info = msgs.get_receive_info();
mtx.unlock();
    }
    visualize.show_receive_info(receive_info);
}


void sentry(Msgs &msgs) {
    cv::VideoCapture    cap("../vedio/05_25--16_36_51.avi");  // cap捕捉图片流
    cv::Mat             img;
    // 检测是否有免驱相机
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return ;
    }
    cap.read(img);

    // 
    GetWarpMatrixConfig get_warp_matrix_config(
        click_point_method,
        team_color
    );
    GetWarpMatrix       get_warp_matrix(get_warp_matrix_config);
    cv::Mat warp_matrix = get_warp_matrix.get_warp_matrix(img);

    // 
    std::string     engine_path = "../module/model/weights/77_last.engine";
    Yolov5TRT       yolov5_net(engine_path);

    // 
    PositionConfig   position_config(
        fix_position_method,
        watchtower_high,
        half_car_high,
        offset_x,
        offset_y
    );
    Position         position(position_config, warp_matrix, img);

    // 
    WatchtowerConfig tower_config(
        watchtower_identity,
        merge_same_car_distance,
        judge_same_color_num_mode_frames_thershold,
        judge_same_color_num_mode_conditions,
        judge_discoloration_car_num_frames_thershold,
        team_color,
        judge_discoloration_car_num_distance,
        judge_emeny_car_distance
    );
    Watchtower       tower(tower_config);

    // 
    std::vector<Yolo::Detection>    yolo_detection;
    std::vector<CarsInfo>           cars_info;
    WatchtowerInfo                  tower_info;


    while (true) {
        // 刷新img
        cap.read(img);

        yolo_detection  = yolov5_net(img);

        cars_info       = position.get_cars_info(yolo_detection);

        if (watchtower_identity == 0) { // 主哨岗
mtx.lock();
            tower_info  = tower.get_watchtower_info(cars_info, receive_info.receive_tower_info, receive_info.receive_car_1_info, receive_info.receive_car_2_info);
mtx.unlock();
        } else if (watchtower_identity == 1) {// 副哨岗
            tower_info  = tower.get_watchtower_info(cars_info);
        }

        msgs.send_info(tower_info);

        visualize.show_img(img, yolo_detection);
        visualize.show_visual_map(cars_info);
        visualize.show_watchtower_info(tower_info);
        visualize.show_receive_info(receive_info);
    }
}

int main() {
    fs_read["watchtower_identity"]            >> watchtower_identity;
    fs_read["team_color_"]                    >> team_color;
    fs_read["fix_position_method_"]           >> fix_position_method;
    fs_read["click_point_method_"]            >> click_point_method;
    fs_read["watchtower_high_"]               >> watchtower_high;
    fs_read["half_car_high_"]                 >> half_car_high;
    fs_read["offset_x_"]                      >> offset_x;
    fs_read["offset_y_"]                      >> offset_y;
    fs_read["merge_same_car_distance_"]                         >> merge_same_car_distance;
    fs_read["judge_same_color_num_mode_frames_thershold_"]      >> judge_same_color_num_mode_frames_thershold;
    fs_read["judge_same_color_num_mode_conditions_"]            >> judge_same_color_num_mode_conditions;
    fs_read["judge_discoloration_car_num_frames_thershold_"]    >> judge_discoloration_car_num_frames_thershold;
    fs_read["judge_discoloration_car_num_distance_"]            >> judge_discoloration_car_num_distance;
    fs_read["judge_emeny_car_distance_"]                        >> judge_emeny_car_distance;
    fs_read["tower_self_ip_"]                                   >> tower_self_ip;
    fs_read["other_tower_ip_"]                                  >> other_tower_ip;
    fs_read["car_1_ip_"]                                        >> car_1_ip;
    fs_read["car_2_ip_"]                                        >> car_2_ip;

    MsgsConfig msgs_config = MsgsConfig(
        watchtower_identity,
        tower_self_ip,
        other_tower_ip,
        car_1_ip,
        car_2_ip
    );
    Msgs        msgs(msgs_config);

    if (watchtower_identity == 0) {
        std::thread t2  = std::thread(receive, std::ref(msgs));
        std::thread t1  = std::thread(sentry, std::ref(msgs));

        t2.detach();
        t1.join();
    } else if (watchtower_identity == 1) {
        sentry(msgs);
    }

    fs_read.release();

    return 0;
}