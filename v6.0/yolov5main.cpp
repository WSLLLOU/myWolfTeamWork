#include "yolov5.hpp"
#include "TRTModule.hpp"
#include "Monitoring.hpp"
#include "Mapinfo.hpp"
#include "Message.hpp"
#include <thread>
#include <mutex>
#include <chrono>
#include <zmq.hpp>

#include "mv_video_capture.hpp"
// #include "fps.hpp"

// #define MAPINFO_OFF
// #define SHOW_IMG
// #define SHOW_RUNTIME

struct Send{			// 套接字内容

    // int gray_num;           // 当前帧灰车的数量

    int swapColorModes;     // 交换颜色模式: 交换后异号异色车--0  交换后同号同色车--1

    cv::Point2f blue1;
    cv::Point2f blue2;
    cv::Point2f red1;
    cv::Point2f red2;

    Send(int swapColorModes, cv::Point2f blue1, cv::Point2f blue2, cv::Point2f red1, cv::Point2f red2) {
        this->swapColorModes = swapColorModes;
        this->blue1 = blue1;
        this->blue2 = blue2;
        this->red1  = red1;
        this->red2  = red2;
    }
};

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
		    //计算变换矩阵
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

std::mutex  mtx;
CarInfoSend PC_2 = CarInfoSend{};
bool        receive;

void start_2(zmq::socket_t& subscriber) {
    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 休眠20ms
        // 获取副哨岗的信息
        zmq::message_t recv_message(sizeof(CarInfoSend));
mtx.lock();
        receive = subscriber.recv(&recv_message, ZMQ_DONTWAIT);
        memcpy(&PC_2, recv_message.data(), sizeof(PC_2));
mtx.unlock();

        std::cout << "是否接受到: " << receive << std::endl;
        // std::cout << "blue1:    " << PC_2.blue1 << std::endl;
        // std::cout << "red2:     " << PC_2.red2 << std::endl;
    }
}


void start_1() {
    std::string engine_name = "/home/wsl/wolf_workspace/tensorrtx/yolov5/imgsz1280_1280_65_b.engine";
    std::string img_dir     = "/home/wsl/wolf_workspace/yoloCustomData/3月19/3月19_right_1.avi";

    cv::VideoCapture    cap(img_dir);  // cap捕捉图片流
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);

    std::vector<car>    result;        // 捕获结果 [模型推理后的信息处理结果 --> 详情看car结构体]
    TRTX                wsl(engine_name);
    // TRTModule           model("/home/wsl/myWolfTeamWorking/v6.0/opt4/model-opt-4.onnx");
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
        return ;
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
    static CarInfoSend PC_1;                // 要传输的数据结构变量
    
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

        std::vector<bbox_t> armor;
        // armor = model(img);                     // 检测 装甲板 opt4

        wolfEye.run(car, armor, img, result);   // 得出结果 ————> result 

#ifndef MAPINFO_OFF
        mapInfo.showTransformImg(img);                          // ++ 显示透视变换后的图片   
        mapInfo.showMapInfo(result);                            // ++ 显示模拟地图    
#endif  // MAPINFO_OFF

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
#endif // SHOW_IMG

mtx.lock();
        // mapInfo.showMapInfo2(PC_2);
        // 获取需要发送的消息内容 // receive: 接收到数据,则融合 || 未接收到,则跳过
        PC_1 = chong(result, PC_2, receive);
mtx.unlock();
        mapInfo.showMapInfo2(PC_1);
        Send PC_1_Send(PC_1.swapColorModes, PC_1.blue1, PC_1.blue2, PC_1.red1, PC_1.red2);                       // 要传输的数据结构变量_2

        flip_vertical(PC_1_Send.blue1.y);
        flip_vertical(PC_1_Send.blue2.y);
        flip_vertical(PC_1_Send.red1.y);
        flip_vertical(PC_1_Send.red2.y);
        
        std::cout << "发送的内容是" << std::endl;
        if (PC_1_Send.swapColorModes == 1) {
            std::cout << "----------------------------------------------------同色同号了------------------" << std::endl;
        }
        std::cout << "blue1:    " << PC_1_Send.blue1 << std::endl;
        std::cout << "blue2:    " << PC_1_Send.blue2 << std::endl;
        std::cout << "red1:     " << PC_1_Send.red1 << std::endl;
        std::cout << "red2:     " << PC_1_Send.red2 << std::endl;

        // 发送消息
        // zmq::message_t  send_message(sizeof(CarInfoSend));
        zmq::message_t  send_message(sizeof(Send));
        // memcpy(send_message.data(), &PC_1, sizeof(PC_1));
        memcpy(send_message.data(), &PC_1_Send, sizeof(PC_1_Send));
        publisher.send(send_message);
        

        if (cv::waitKey(1) == 'q') {
            break;
        }
        mv_capture_.cameraReleasebuff();   // 释放这一帧的内容
#ifndef SHOW_RUNTIME
        auto end = std::chrono::system_clock::now();
        std::cout << "整体时间" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
#endif  // SHOW_RUNTIME
    }
    cap.release();
}

int main(int argc, char **argv) {
    // 初始化 接收
    zmq::context_t receive_context(1);
    zmq::socket_t subscriber(receive_context, ZMQ_SUB);
    subscriber.connect("tcp://192.168.1.147:6666"); //147
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    std::thread t1 = std::thread(start_2, std::ref(subscriber));
    std::thread t2 = std::thread(start_1);

    t1.detach();
    t2.join();

    return 0;
}