#include "yolov5.hpp"

// 标4点位置切换
// 0 点四边玻璃罩子
// 1 点中间各障碍物左下点
#define CHECK_4_OPT 0
// 数据矫正函数模式
// 0  蓝方主哨岗
// 1  蓝方副哨岗
// 2  红方主哨岗
// 3  红方副哨岗
#define MOTHED 0
// 当前身份 蓝/红
// "blue"   现在为蓝方    记得一定要小写,并且内容无误
// "red"    现在为红方    记得一定要小写,并且内容无误
#define WHO_AM_I "blue"

// 哨岗相机离地高度 (mm)
#define WATCH_DOG_H 1730.0
// 车半身高度 (mm)
#define CAR_HALF_H 250.0
// 相机偏移量 X (cm)
#define OFFSET_X 9
// 相机偏移量 Y (cm)
#define OFFSET_Y 12

// 显示虚拟地图
// 0 关闭
// 1 开启
#define SHOW_MAPINFO 1
// 显示相机图片(并且绘制了, 标志数据和坐标数据)
// 0 关闭
// 1 开启
#define SHOW_IMG 1
// 录制
// 0 录制关闭
// 1 录制开启
#define WRITER 1
// 录制视频目标路径
#define WRITER_DIR "../vedio/"

#include "Monitoring.hpp"
#include "Mapinfo.hpp"
#include "Message.hpp"
#include <mutex>
#include <chrono>
#include <zmq.hpp>
#include <time.h>
#include <sstream>
#include "mv_video_capture.hpp"
// #include "fps.hpp"

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
		    cv::Point2f god_view[4];

#if CHECK_4_OPT == 0
            if (WHO_AM_I == "blue") {
                god_view[0] = cv::Point2f(100, 808-638);
                god_view[1] = cv::Point2f(348, 100);
                god_view[2] = cv::Point2f(100, 708);
                god_view[3] = cv::Point2f(348, 808-150);
                // god_view[] 	= { cv::Point2f(100, 808-638), cv::Point2f(348, 100), cv::Point2f(100, 708), cv::Point2f(348, 808-150) };
            }
		    if (WHO_AM_I == "red") {
                god_view[0] = cv::Point2f(100,      808-638);
                god_view[1] = cv::Point2f(348,      100);
                god_view[2] = cv::Point2f(100+20,   708);
                god_view[3] = cv::Point2f(348,      808-150);
                // god_view[] 	    = { cv::Point2f(100, 808-638), cv::Point2f(348, 100), cv::Point2f(100+20, 708), cv::Point2f(348, 808-150) };
            }
#endif
#if CHECK_4_OPT == 1
                god_view[0] = cv::Point2f(214,      808-578);
                god_view[1] = cv::Point2f(93.5,     808-354);
                god_view[2] = cv::Point2f(334.5,    808-354);
                god_view[3] = cv::Point2f(214,      808-150);
#endif

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

#if WRITER == 1
    time_t vedio_name = time(NULL);
    tm* tm_t = localtime(&vedio_name);
    std::stringstream vns;
    vns << std::to_string(tm_t->tm_hour) << "_" << std::to_string(tm_t->tm_min) << "_" << std::to_string(tm_t->tm_sec);
    // 录制
    cv::VideoWriter writer;
    std::string out_path = WRITER_DIR + vns.str() +".avi";    // 目标路径
    cv::Size size(1280, 1024);                              // 重要! 要求与摄像头参数一致
    // int fourcc = writer.fourcc('X', 'V', 'I', 'D');      // 设置avi文件对应的编码格式 66 67
    int fourcc = writer.fourcc('M', 'J', 'P', 'G');     // 33 30 48Flv1
    // int fourcc = writer.fourcc('F', 'L', 'V', '1');      // 33 30 48Flv1
    writer.open(out_path, fourcc, 30, size, true);      // CAP_DSHOW = true
    if (writer.isOpened()) {
        std::cout << "正在录制" << std::endl;
    }
    else {
        std::cout << "录制失败" << std::endl;
    }
#endif  // WRITER_OFF

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
        auto start = std::chrono::system_clock::now();

        if (mv_capture_.isindustryimgInput()) {
            img = mv_capture_.image();
        }
        else {
            cap.read(img);
        }

#if WRITER == 1
        writer << img;
#endif  // WRITER_OFF

        auto car = wsl(img);                    // 检测 车车

        wolfEye.run(car, img, result);   // 得出结果 ————> result 

#if SHOW_MAPINFO == 1
        // mapInfo.showTransformImg(img);                          // ++ 显示透视变换后的图片
        mapInfo.showMapInfo(result);                            // ++ 显示模拟地图
#endif  // SHOW_MAPINFO

        // 获取需要发送的消息内容 // receive: 接收到数据,则融合 || 未接收到,则跳过
        PC_2 = chong(result);

// #if SHOW_MAPINFO == 1
//         mapInfo.showMapInfo2(PC_2); // 显示融合后的数据
// #endif  // SHOW_MAPINFO

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

        auto end = std::chrono::system_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "整体时间" << diff << "ms" << std::endl << std::endl;

#if SHOW_IMG == 1
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
        cv::putText(img, "buff_1_        : " + std::to_string(PC_2.a_dog_in_the_toilet_on_shit_1),         cv::Point(50, 150), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, "buff_2_        : " + std::to_string(PC_2.a_dog_in_the_toilet_on_shit_2),         cv::Point(50, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, "FPS            : " + std::to_string(1000.0/diff),                                cv::Point(50, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);

        cv::putText(img, "blue1          :", cv::Point(50, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.blue1.x)),         cv::Point(350, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.blue1.y)),         cv::Point(450, 550), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, "blue2:         :", cv::Point(50, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.blue2.x)),         cv::Point(350, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.blue2.y)),         cv::Point(450, 600), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, "red1           :", cv::Point(50, 650), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.red1.x)),          cv::Point(350, 650), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.red1.y)),          cv::Point(450, 650), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, "red2           :", cv::Point(50, 700), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.red2.x)),          cv::Point(350, 700), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);
        cv::putText(img, std::to_string(int(PC_2.red2.y)),          cv::Point(450, 700), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0x00, 0x00, 0xFF), 2);

    // 在原图上绘制检测结果 end
        cv::imshow("yolov5", img);
        static char _key;
        _key = cv::waitKey(1);
        if (_key == 'q') {
            break;
        }
#endif // SHOW_IMG

    }

#if WRITER == 1
    writer.release();
#endif  // WRITER_OFF

    cap.release();

    return 0;
}