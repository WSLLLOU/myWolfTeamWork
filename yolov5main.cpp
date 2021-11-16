// #include "yolov5.hpp"
// #include "car_position.hpp"
// #include "map_info.hpp"

// int main(int argc, char **argv) {
//     std::string engine_name = "/home/wsl/wolf_workspace/tensorrtx_v5.0/v5.0/build/engine/m43best_25epochs.engine";
//     std::string img_dir     = "/home/wsl/wolf_workspace/RMUA/Bilibili_ICRA/2.mp4";

//     cv::VideoCapture cap(img_dir);  // cap捕捉图片流

//     start(engine_name);             // yolov5.hpp
    
//     // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
// 	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);

//     // 检测是否有相机
//     if (!cap.isOpened()) {
//         std::cout << "Error opening video stream or file" << std::endl;
//         return -1;
//     }

//     cv::Mat img;
//     cap.read(img);
//     cv::Mat warpmatrix = getTransformMask(img);         // ++ 获得透视变换矩阵      car_position
//     initMap();                                          // ++ 初始化模拟地图        map_info

//     while (true)
//     {
//         cap.read(img);
//         showTransformImg(warpmatrix, img);              // ++ 显示透视变换后的图片   car_position
//         showMapInfo();                                  // ++ 显示化模拟地图        map_info
//         auto res = yolov5_v5_Rtx_start(img);
//         // 绘制 矩形(rectangle) 和 类编号(class_id)
//         for (size_t j = 0; j < res.size(); j++)  {      // res.size() 该图检测到多少个class
//             cv::Rect r = get_rect(img, res[j].bbox);
//             cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
//             cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//             drawCarPosition(res[j], warpmatrix, img);   // ++ 在模拟地图上画车车     map_info
//         }

//         cv::imshow("yolov5", img);
//         if (cv::waitKey(1) == 'q') {
//             destroy();                                  // yolov5.hpp
//             break;
//         }
//     }
// }


#include "TRTModule.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

int main() {
    TRTModule model("/home/wsl/wolf_workspace/tensorrtx_v5.0/v5.0/opt4/model-opt-4.onnx");
    
    std::string img_dir     = "/home/wsl/下载/哨岗3亮度稍暗暗.avi";
    // int img_dir = 0;
    cv::VideoCapture cap(img_dir);  // cap捕捉图片流
    
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);

    // 检测是否有相机
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    cv::Mat img;
    const cv::Scalar colors[4] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}, {255, 255, 255}};

    while (true)
    {
        cap.read(img);
        // auto start = std::chrono::system_clock::now();
        auto detections = model(img);
        // auto end = std::chrono::system_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        /* show detections */
        if (!detections.empty()) {
            // cv::Mat im2show = img.clone();
            // for (const auto &b: detections) {
            for (int i = 0; i < detections.size(); i++) {
                cv::line(img, detections[i].pts[0], detections[i].pts[1], colors[2], 2);
                cv::line(img, detections[i].pts[1], detections[i].pts[2], colors[2], 2);
                cv::line(img, detections[i].pts[2], detections[i].pts[3], colors[2], 2);
                cv::line(img, detections[i].pts[3], detections[i].pts[0], colors[2], 2);
                cv::putText(img, std::to_string(detections[i].tag_id), detections[i].pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[detections[i].color_id]);
                // std::cout << armor.img_center_dist << std::endl;
            }
        }
        cv::imshow("yolov5", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }

    }

    return 0;
}