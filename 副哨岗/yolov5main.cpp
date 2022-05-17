#include "yolov5.hpp"
#include "Monitoring.hpp"
#include "Mapinfo.hpp"
#include "Message.hpp"
#include <mutex>
#include <chrono>
#include <zmq.hpp>

#include "mv_video_capture.hpp"
// #include "fps.hpp"

// #define MAPINFO_OFF
// #define SHOW_IMG
// #define SHOW_RUNTIME

// 旋转矩阵
cv::Mat warpmatrix(3, 3, CV_64FC1);
// 回调函数点击事件
static void onMouse1(int event, int x, int y, int, void* userInput) {
    static int          times   = 0;
    static cv::Point2f  fourPoint[4];
	if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }
    else {
        times++;
	    if (times <= 4) {
            std::cout << x << "  " << y << std::endl;
            fourPoint[times-1].x = x;
            fourPoint[times-1].y = y;
        }
        else if (times == 5) {
		    // cv::Point2f god_view[] 	  = { cv::Point2f(808,448), cv::Point2f(808,0), cv::Point2f(0,448), cv::Point2f(0,0) };
		    cv::Point2f god_view[] 	    = { cv::Point2f(100, 808-638), cv::Point2f(348, 100), cv::Point2f(100, 708), cv::Point2f(348, 808-150) };
		    // 计算变换矩阵
		    warpmatrix = cv::getPerspectiveTransform(fourPoint, god_view);
		    std::cout << warpmatrix << std::endl;
        }
    }
}
// 获取透视变换矩阵
void getWarpMatrix(cv::Mat& img) {
    while(true) {
        cv::imshow("test", img);
        cv::setMouseCallback("test", onMouse1); // 调用回调函数
        if (cv::waitKey(1) == 'q') {
            cv::destroyWindow("test");
            break;
        }
    }
}

CarInfoSend PC_2 = CarInfoSend{};

int main() {
    std::string engine_name = "/home/wsl/wolf_workspace/tensorrtx/yolov5/imgsz1280_1280_65_b.engine";
    std::string img_dir     = "/home/wsl/wolf_workspace/yoloCustomData/5000曝光2.avi";

    cv::VideoCapture    cap(img_dir);  // cap捕捉图片流
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);

    std::vector<car>    result;        // 捕获结果 [模型推理后的信息处理结果 --> 详情看car结构体]
    TRTX                wsl(engine_name);
    cv::Mat             img;

    mindvision::VideoCapture mv_capture_ = mindvision::VideoCapture(
                                                    mindvision::CameraParam(0,          // 0--工业相机, 1--其他免驱相机
                                                                            mindvision::RESOLUTION_1280_X_1024,
                                                                            mindvision::EXPOSURE_NONE
                                                                            )
                                                                        );
    // fps::FPS       global_fps_;

    // 检测是否有免驱相机
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }
    if (mv_capture_.isindustryimgInput()) {
        img = mv_capture_.image();
    }
    else {
        cap.read(img);
    }

    getWarpMatrix(img);                     // 手动点四点透视变换

    Monitoring wolfEye(warpmatrix);         // 哨岗类
    MapInfo mapInfo(wolfEye.getmatrix());   // 俯视图显示类
    Message chong;                          // 传输数据处理类
    static CarInfoSend PC_2;                // 要传输的数据结构变量
    
    // 初始化 发送
    zmq::context_t  ip_context(1);
    zmq::socket_t   publisher(ip_context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5556");


    while (true) {
#ifndef SHOW_RUNTIME
        auto start = std::chrono::system_clock::now();
#endif  // SHOW_RUNTIME
        if (mv_capture_.isindustryimgInput()) {
            img = mv_capture_.image();
        }
        else {
            cap.read(img);
        }

        auto car = wsl(img);                    // 检测 车车

        wolfEye.run(car, img, result);   // 得出结果 ————> result 

#ifndef MAPINFO_OFF
        // mapInfo.showTransformImg(img);                          // ++ 显示透视变换后的图片
        mapInfo.showMapInfo(result);                            // ++ 显示模拟地图
#endif  // MAPINFO_OFF

        // 获取需要发送的消息内容 // receive: 接收到数据,则融合 || 未接收到,则跳过
        PC_2 = chong(result);

#ifndef MAPINFO_OFF
        mapInfo.showMapInfo2(PC_2); // 显示融合后的数据
#endif  // MAPINFO_OFF

        std::cout << "发送的内容是" << std::endl;
        // std::cout << "swapColorModes:" << PC_2.swapColorModes  << std::endl;
        // std::cout << "pangolin:      " << PC_2.pangolin        << std::endl;
        std::cout << "buff_1_:       " << PC_2.a_dog_in_the_toilet_on_shit_1 << std::endl;
        std::cout << "buff_2_:       " << PC_2.a_dog_in_the_toilet_on_shit_2 << std::endl;
        std::cout << "blue1:         " << PC_2.blue1           << std::endl;
        std::cout << "blue2:         " << PC_2.blue2           << std::endl;
        std::cout << "red1:          " << PC_2.red1            << std::endl;
        std::cout << "red2:          " << PC_2.red2            << std::endl;
        std::cout << "gray1:         " << PC_2.gray_1          << std::endl;
        std::cout << "gray2:         " << PC_2.gray_2          << std::endl;
        std::cout << "gray3:         " << PC_2.gray_3          << std::endl;
        std::cout << "gray4:         " << PC_2.gray_4          << std::endl;

        // 发送消息
        zmq::message_t send_message(sizeof(CarInfoSend));
        memcpy(send_message.data(), &PC_2, sizeof(PC_2));
        publisher.send(send_message);

        mv_capture_.cameraReleasebuff();   // 释放这一帧的内容
#ifndef SHOW_RUNTIME
        auto end = std::chrono::system_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "整体时间" << diff << "ms" << std::endl << std::endl;
#endif  // SHOW_RUNTIME

#ifndef SHOW_IMG
    // 在原图上绘制检测结果 start
        // 绘制 矩形(rectangle) 和 类编号(class_id)
        for (size_t j = 0; j < car.size(); j++) {               // car.size() 该图检测到多少个class
            // if( car[j].class_id == 0) {
                cv::Rect r  = get_rect(img, car[j].bbox);
                // r           = r + cv::Point(0, r.height/2);       //平移，左上顶点的 `x坐标`不变，`y坐标` +r.height/2
                // r           = r + cv::Size (0, -r.height/2);      //缩放，左上顶点不变，宽度不变，高度减半
                cv::rectangle(img, r, cv::Scalar(0xff, 0xff, 0xff), 2);
                cv::putText(img, std::to_string((int)car[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            // }
        }
        cv::putText(img, "buff_1_        : " + std::to_string(PC_2.a_dog_in_the_toilet_on_shit_1),         cv::Point(50, 150), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        cv::putText(img, "buff_2_        : " + std::to_string(PC_2.a_dog_in_the_toilet_on_shit_2),         cv::Point(50, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        cv::putText(img, "FPS            : " + std::to_string(1000.0/diff),                                cv::Point(50, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
/*
        // 在原图上画装甲板opt4
        const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
        for (const auto &b : armor) {
            cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
            cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
            cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
            cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
            cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
        }
*/
    // 在原图上绘制检测结果 end
        cv::imshow("yolov5", img);
        static char _key;
        _key = cv::waitKey(1);
        if (_key == 'q') {
            break;
        }
#endif // SHOW_IMG

    }
    cap.release();

    return 0;
}