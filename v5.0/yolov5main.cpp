#include "yolov5.hpp"
#include "TRTModule.hpp"
#include "Monitoring.hpp"
#include "Mapinfo.hpp"
#include "Message.hpp"
#include <zmq.hpp>

#define MAPINFO_OFF
#define SHOW_IMG

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
		    cv::Point2f god_view[] 	    = { cv::Point2f(100, 808-638), cv::Point2f(348, 100), cv::Point2f(120, 708), cv::Point2f(348, 808-150) };
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


int main(int argc, char **argv) {
    std::string engine_name = "/home/wsl/wolf_workspace/tensorrtx_v5.0/v5.0/build/engine/m43best_25epochs.engine";
    std::string img_dir     = "/home/wsl/下载/哨岗2亮度稍暗.avi";

    cv::VideoCapture    cap(img_dir);  // cap捕捉图片流
    std::vector<car>    result;        // 捕获结果 [模型推理后的信息处理结果 --> 详情看car结构体]
    TRTX                wsl(engine_name);
    TRTModule           model("/home/wsl/wolf_workspace/tensorrtx_v5.0/v5.0/opt4/model-opt-4.onnx");
    cv::Mat             img;
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    // 检测是否有相机
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }
    cap.read(img);
    getWarpMatrix(img);

    Monitoring wolfEye(warpmatrix);         // 哨岗类
    MapInfo mapInfo(wolfEye.getmatrix());   // 俯视图显示类
    Message chong;                          // 传输数据处理类
    static CarPositionSend chongXY;         // 要传输的数据结构变量
    /*
    // plz duguxiaochong program
    zmq::context_t  ip_context(1);
    zmq::socket_t   publisher(ip_context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5556");
    zmq::message_t  send_message(sizeof(CarPositionSend));
    */

    
    while (true) {
        auto start = std::chrono::system_clock::now();

        cap.read(img);
        auto car = wsl(img);                    // 检测 车车
        auto armor = model(img);                // 检测 装甲板 opt4

        wolfEye.run(car, armor, img, result);   // 得出结果 ————> result 

#ifndef MAPINFO_OFF
        // 
        mapInfo.showTransformImg(img);                          // ++ 显示透视变换后的图片   
        mapInfo.showMapInfo(result);                            // ++ 显示模拟地图    

    // 在原图上绘制检测结果 start
        // 绘制 矩形(rectangle) 和 类编号(class_id)
        for (size_t j = 0; j < car.size(); j++) {               // car.size() 该图检测到多少个class
            if( car[j].class_id == 0) {
                cv::Rect r = get_rect(img, car[j].bbox);
                cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(img, std::to_string((int)car[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
        }

        // 在原图上画装甲板opt4
        const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
        for (const auto &b : armor) {
            cv::line(img, b.pts[0], b.pts[1], colors[2], 2);
            cv::line(img, b.pts[1], b.pts[2], colors[2], 2);
            cv::line(img, b.pts[2], b.pts[3], colors[2], 2);
            cv::line(img, b.pts[3], b.pts[0], colors[2], 2);
            cv::putText(img, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
        }
    // 在原图上绘制检测结果 end
#endif  // MAPINFO_OFF

#ifndef SHOW_IMG
        cv::imshow("yolov5", img);
#endif // SHOW_IMG

        // for (auto i = result.begin(); i != result.end(); i++) {
        //     if ((*i).color == 0) {
        //         std::cout << "    car_color:    blue" << std::endl;
        //     }
        //     else if ((*i).color == 1) {
        //         std::cout << "    car_color:    red" << std::endl;
        //     }
        //     else if ((*i).color == 2) {
        //         std::cout << "    car_color:    gray" << std::endl;
        //     }
        //     else if ((*i).color == -1) {
        //         std::cout << "    car_color:    I don't know" << std::endl;
        //     }
        //     std::cout << "      car_num:    " << (*i).num << std::endl;
        //     std::cout << "carPosition_x:    " << (*i).carPosition.x << std::endl;
        //     std::cout << "carPosition_y:    " << (*i).carPosition.y << std::endl;
        //     std::cout << "carPositionFix_x: " << (*i).carPositionFixed.x << std::endl;
        //     std::cout << "carPositionFix_y: " << (*i).carPositionFixed.y << std::endl;
        //     std::cout << std::endl;
        // }

        chongXY = chong(result);

        /*
        // send(chongXY);
        // plz duguxiaochong program
        memcpy(send_message.data(), &chongXY, sizeof(chongXY));
        publisher.send(send_message);
        */

        if (cv::waitKey(1) == 'q') {
            break;
        }

        auto end = std::chrono::system_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }
}