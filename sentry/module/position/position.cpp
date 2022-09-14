#include "position.hpp"

Position::Position(int fix_position_method, cv::Mat warp_matrix, cv::Mat src_img) {
    this->fix_position_method_  = fix_position_method;
    this->warp_matrix_          = warp_matrix;
    this->tool_img_             = src_img;
}

Position::~Position() {
    this->cars_info_.clear();
    this->armors_info_.clear();
}

// 
void Position::yoloDetection2CarsInfo(std::vector<Yolo::Detection> &predicts) {
    this->cars_info_.clear();
    this->cars_info_.shrink_to_fit();
    this->armors_info_.clear();
    this->armors_info_.shrink_to_fit();

    // 按置信度升序
    std::sort(predicts.begin(), predicts.end(), [&](const Yolo::Detection& predicts_1, const Yolo::Detection& predicts_2){
        return predicts_1.conf < predicts_2.conf;
    });

    // predicts 分到 cars_info 和 armors_info
    for (Yolo::Detection &predict : predicts) { // predicts.size() 该图检测到多少个物体
        // cars
        static CarsInfo temp_car;
        static cv::Rect temp_r;
        temp_r  = get_rect(this->tool_img_, predict.bbox);

        // armors
        static ArmorsInfo   temp_armor;
        static cv::Point2f  img_armor_center;

        if (predict.class_id == 0) {             // 0 'car'
            temp_car.position       = getWarpPosition(cv::Point2f(temp_r.x+temp_r.width/2, temp_r.y+temp_r.height/2));   // 透视变换矩形 对 车体检测框中心点 进行坐标转化 
            temp_car.color          = -1;   // 初始化 -1
            temp_car.num            = -1;   // 初始化 -1
            // 修改矩形框的大小，基于左下角垂直缩小一半，便于分类装甲板的归属
            temp_r                  = temp_r + cv::Point(0, temp_r.height/2);       //平移，左上顶点的 `x坐标`不变，`y坐标` +temp_r.height/2
            temp_r                  = temp_r + cv::Size (0, -temp_r.height/2);      //缩放，左上顶点不变，宽度不变，高度减半
            temp_car.img_rect       = temp_r;
            // 
            this->cars_info_.push_back(temp_car);
        }
        else {
            // img_armor_center
            img_armor_center = getCenterPoint(get_rect(this->tool_img_, predict.bbox));
            temp_armor.img_armor_center = img_armor_center;

            // color && armor_num
            // color    // 0蓝 1红 2黑
            // num      // 1 / 2
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
                temp_armor.num      = -1;       // 主不关心死车的号数
            }

            this->armors_info_.push_back(temp_armor);
        }
    }

    // rect.contains(cv::Point(x, y));          //返回布尔变量，判断rect是否包含Point(x, y)点
    // center (r.x+r.width/2, r.y+r.height/2)
    // 判断 [car 的矩形框[] 中是否有 [armor 的中心点] 存在
    for (auto i = this->cars_info_.begin(); i != this->cars_info_.end(); i++) {
        for (auto j = this->armors_info_.begin(); j != this->armors_info_.end(); j++) {
            if( (*i).img_rect.contains((*j).img_armor_center) ) {
                (*i).color = (*j).color;
                (*i).num = (*j).num;
                break;
            }
        }
    }
}

// 透视变换点
cv::Point2f Position::getWarpPosition(const cv::Point2f &ptOrigin) {
	cv::Mat_<double> matPt(3, 1);
	matPt(0, 0) = ptOrigin.x;
	matPt(1, 0) = ptOrigin.y;
	matPt(2, 0) = 1;
	cv::Mat matPtView = this->warp_matrix_ * matPt;
	double x = matPtView.at<double>(0, 0);
	double y = matPtView.at<double>(1, 0);
	double z = matPtView.at<double>(2, 0);
    
	return cv::Point2f(x * 1.0 / z, y * 1.0 / z);
}

// 求旋转矩形四点中心
cv::Point2f Position::getCenterPoint(const cv::Point2f pts[]) {
    static cv::Point2f center;
    center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4.0;
    center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4.0;
    return center;
}
// 求矩形中心点
cv::Point2f Position::getCenterPoint(const cv::Rect &rect) {
    static cv::Point2f center;
    center.x = rect.x + cvRound(rect.width  / 2.0);
    center.y = rect.y + cvRound(rect.height / 2.0);
    return center;
}


void Position::fixCarsPosition(cv::Point2f& car_position) {
    // b_1 == 0  蓝方主哨岗
    // b_2 == 1  蓝方副哨岗
    // r_1 == 2  红方主哨岗
    // r_2 == 3  红方副哨岗

    // 坐标轴翻转 至 能够合适地进行坐标误差矫正
    if (this->fix_position_method_ == 0) {
        flipVertical(car_position.y);
    }
    else if (this->fix_position_method_ == 1) {
        flipVertical(car_position.y);
    }
    else if (this->fix_position_method_ == 2) {
        flipDiagonal(car_position.x, car_position.y);
    }
    else if (this->fix_position_method_ == 3) {
        flipDiagonal(car_position.x, car_position.y);
    }

    // 坐标误差矫正
    static float watchDog   = 1730.0; // WATCH_DOG_H;   // 1768 -- 20cm*20cm*20cm -- 5kg
    static float carHeight  = 250.0;  // CAR_HALF_H;
    static float nicetry    = carHeight / watchDog;
    static float offset_x   = 9;   // OFFSET_X;
    static float offset_y   = 12;  // OFFSET_Y;
    car_position.x = (car_position.x + offset_x) * (1.0 - nicetry);
    car_position.y = (car_position.y + offset_y) * (1.0 - nicetry);

    // 坐标轴翻转 至 roboCar需要的坐标轴
    if (this->fix_position_method_ == 0) {
        flipVertical(car_position.y);
    }
    else if (this->fix_position_method_ == 1) {
        flipHorizontal(car_position.x);
    }
    else if (this->fix_position_method_ == 2) {
        // 不需要做任何操作
    }
    else if (this->fix_position_method_ == 3) {
        flipDiagonal(car_position.x,      car_position.y);
    }
    // std::swap(carPosition.x,      carPosition.y);
    // std::swap(carPositionFixed.x, carPositionFixed.y);
}

std::vector<CarsInfo> Position::get_cars_info(std::vector<Yolo::Detection> &predicts) {
    yoloDetection2CarsInfo(predicts);
    for (CarsInfo &car : this->cars_info_) {
        fixCarsPosition(car.position);
    }

    return this->cars_info_;
}