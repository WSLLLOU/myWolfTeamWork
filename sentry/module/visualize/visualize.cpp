#include "visualize.hpp"


void Visualize::show_img(const cv::Mat &src) {
    cv::imshow("show_img_src", src);
    cv::waitKey(1);
}

void Visualize::show_img(cv::Mat &src, std::vector<Yolo::Detection> yolo_detection) {
    // static const cv::Scalar colors[] = {{0,0,0}, {255,0,0}, {0,0,255}, {0,0,0}};
    // 绘制 矩形(rectangle) 和 类编号(class_id)
    for (Yolo::Detection &detection : yolo_detection) {
        cv::Rect r  = get_rect(src, detection.bbox);
        cv::rectangle(src, r, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        cv::putText(src, std::to_string((int)detection.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x00, 0xFF, 0x00), 2);
    }
    cv::imshow("show_img_src_detection", src);
    cv::waitKey(1);
}

void Visualize::show_img(cv::Mat &src, cv::Mat warp_matrix) {
    static cv::Mat warp_src;
    cv::warpPerspective(src, warp_src, warp_matrix, cv::Size(448, 808),cv::INTER_LINEAR);
    cv::imshow("show_img_warp_src", warp_src);
    cv::waitKey(1);
}

void Visualize::show_visual_map(std::vector<CarsInfo> &cars_info) {
    static       cv::Mat    map;
    static const cv::Scalar colors[] = {{0,0,0}, {255,0,0}, {0,0,255}, {0,0,0}};
    this->background_.copyTo(map);

    for (CarsInfo &car : cars_info) {        
        cv::circle(map,  cv::Point(car.position.x, car.position.y), 25, colors[car.color+1], -1);
        cv::putText(map, std::to_string((int)car.num), cv::Point(car.position.x - 6, car.position.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }

    cv::imshow("show_visual_map_cars_info", map);
    cv::waitKey(1);
}

void Visualize::show_visual_map(WatchtowerInfo &tower_info) {
    static       cv::Mat    map;
    static const cv::Scalar colors[] = {{0,0,0}, {255,0,0}, {0,0,255}, {0,0,0}};
    this->background_.copyTo(map);

    if (tower_info.blue1 != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.blue1, 25, colors[1], -1);
        cv::putText(map, std::to_string(1), cv::Point(tower_info.blue1.x - 6, tower_info.blue1.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.blue2 != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.blue2, 25, colors[1], -1);
        cv::putText(map, std::to_string(2), cv::Point(tower_info.blue2.x - 6, tower_info.blue2.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.red1  != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.red1,  25, colors[2], -1);
        cv::putText(map, std::to_string(1), cv::Point(tower_info.red1.x - 6, tower_info.red1.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.red2 != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.red2,  25, colors[2], -1);
        cv::putText(map, std::to_string(2), cv::Point(tower_info.red2.x - 6, tower_info.red2.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.blue1_temp != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.blue1_temp, 25, colors[1], -1);
        cv::putText(map, std::to_string(1), cv::Point(tower_info.blue1_temp.x - 6, tower_info.blue1_temp.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.blue2_temp != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.blue2_temp, 25, colors[1], -1);
        cv::putText(map, std::to_string(2), cv::Point(tower_info.blue2_temp.x - 6, tower_info.blue2_temp.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.red1_temp  != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.red1_temp,  25, colors[2], -1);
        cv::putText(map, std::to_string(1), cv::Point(tower_info.red1_temp.x - 6, tower_info.red1_temp.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    if (tower_info.red2_temp != cv::Point2f(-1, -1)) {
        cv::circle(map, tower_info.red2_temp,  25, colors[2], -1);
        cv::putText(map, std::to_string(2), cv::Point(tower_info.red2_temp.x - 6, tower_info.red2_temp.y + 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }

    cv::imshow("show_visual_map_tower_info", map);
    cv::waitKey(1);
}


void Visualize::show_watchtower_info(WatchtowerInfo &tower_info) {
    static cv::Mat white = ~ cv::Mat::zeros(808, 808, CV_8UC3);
    static cv::Mat whiteboard;
    white.copyTo(whiteboard);
        cv::putText(whiteboard, "gray_num          : " + std::to_string(tower_info.gray_num),          cv::Point(50, 50),   cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "swap_color_mode   : " + std::to_string(tower_info.swap_color_mode),   cv::Point(50, 100),  cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "discoloration_num : " + std::to_string(tower_info.discoloration_num), cv::Point(50, 150), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "gray_on_buff_F6   : " + std::to_string(tower_info.gray_on_buff_F6),   cv::Point(50, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "gray_on_buff_F1   : " + std::to_string(tower_info.gray_on_buff_F1),   cv::Point(50, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);

        cv::putText(whiteboard, "blue1          :", cv::Point(50, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.blue1.x)),         cv::Point(350, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.blue1.y)),         cv::Point(450, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "blue2:         :", cv::Point(50, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.blue2.x)),         cv::Point(350, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.blue2.y)),         cv::Point(450, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "red1           :", cv::Point(50, 650), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.red1.x)),          cv::Point(350, 650), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.red1.y)),          cv::Point(450, 650), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "red2           :", cv::Point(50, 700), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.red2.x)),          cv::Point(350, 700), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(tower_info.red2.y)),          cv::Point(450, 700), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);

    cv::imshow("show_watchtower_info", whiteboard);
    cv::waitKey(1);
}

void Visualize::show_receive_info(ReceiveInfo &receive_info) {
    static cv::Mat white = ~ cv::Mat::zeros(808, 808, CV_8UC3);
    static cv::Mat whiteboard;
    white.copyTo(whiteboard);

        // cv::putText(whiteboard, "FPS            : " + std::to_string(1000.0/diff),               cv::Point(50, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "other_tower_online?: " + std::to_string(receive_info.receive_tower_info.other_tower_online),   cv::Point(50, 350), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "car_1_OK?          : " + std::to_string(receive_info.receive_car_1_info.comfirm_receipt),      cv::Point(50, 400), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "car_2_OK?          : " + std::to_string(receive_info.receive_car_2_info.comfirm_receipt),      cv::Point(50, 450), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);

        cv::putText(whiteboard, "car1      :", cv::Point(450, 400), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(receive_info.receive_car_1_info.robo_car_info.position.x)),    cv::Point(650, 400), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(receive_info.receive_car_1_info.robo_car_info.position.y)),    cv::Point(750, 400), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, "car2      :", cv::Point(450, 450), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(receive_info.receive_car_2_info.robo_car_info.position.x)),    cv::Point(650, 450), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(whiteboard, std::to_string(int(receive_info.receive_car_2_info.robo_car_info.position.y)),    cv::Point(750, 450), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
    
    cv::imshow("show_receive_info", whiteboard);
    cv::waitKey(1);
}


Visualize::Visualize() {
    // background_ = ~background_;
    cv::Mat roi_B3 = background_(cv::Rect(0,       150,      100, 20)); // cv::Rect(左上角的点 和 宽高)
    cv::Mat roi_B7 = background_(cv::Rect(448-100, 808-170,  100, 20));
    cv::absdiff(roi_B3, B3_B7_, roi_B3);                                // 相减，取绝对值
    cv::absdiff(roi_B7, B3_B7_, roi_B7);

    cv::Mat roi_B1 = background_(cv::Rect(448-120, 0,        20,  100));
    cv::Mat roi_B4 = background_(cv::Rect(100,     808-100,  20,  100));
    cv::Mat roi_B6 = background_(cv::Rect(93,      354,      20,  100));   // 93  -> 93.5
    cv::Mat roi_B9 = background_(cv::Rect(335,     354,      20,  100));   // 335 -> 334.5
    cv::absdiff(roi_B1, B1_B4_B6_B9_, roi_B1);
    cv::absdiff(roi_B4, B1_B4_B6_B9_, roi_B4);
    cv::absdiff(roi_B6, B1_B4_B6_B9_, roi_B6);
    cv::absdiff(roi_B9, B1_B4_B6_B9_, roi_B9);

    cv::Mat roi_B2 = background_(cv::Rect(214,     150,      20,  80));
    cv::Mat roi_B8 = background_(cv::Rect(214,     578,      20,  80));
    cv::absdiff(roi_B2, B2_B8_, roi_B2);
    cv::absdiff(roi_B8, B2_B8_, roi_B8);

    cv::Mat roi_B5 = background_(cv::Rect(224-13,  404-13,   26,  26));
    // 在B5上画个旋转矩形
    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(13,13), cv::Size2f(18, 18), 45);
    cv::Point2f vertices2f[4];          // 定义4个点的数组
    rRect.points(vertices2f);           // 将四个点存储到 `vertices` 数组中
    cv::Point   vertices[4];
    for (int i = 0; i < 4; ++i) {
        vertices[i] = vertices2f[i];
    }
    cv::fillConvexPoly(B5_, vertices, 4, cv::Scalar(255, 255, 255));
    cv::absdiff(roi_B5, B5_, roi_B5);

    background_.copyTo(visual_map_);
}

Visualize::~Visualize() {
}
