#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

// // 标4点位置切换
// // 0 点四边玻璃罩子
// // 1 点中间各障碍物左下点
// #define CHECK_4_OPT 0
// // 当前身份 蓝/红
// // "blue"   现在为蓝方    记得一定要小写,并且内容无误
// // "red"    现在为红方    记得一定要小写,并且内容无误
// #define WHO_AM_I "red"

struct GetWarpMatrixInfo
{
    int     click_point_method_;
    int     temp_color_;
    cv::Mat warp_matrix_;

    GetWarpMatrixInfo(int cpm_, int tc_, cv::Mat wm_) : click_point_method_(cpm_), temp_color_(tc_), warp_matrix_(wm_) {}
};

class GetWarpMatrix {
    private:
        // 标4点位置切换
        // 0 点四边玻璃罩子
        // 1 点中间各障碍物左下点
        int     click_point_method_;
        // 当前身份 蓝/红
        // 0   "blue"   现在为蓝方
        // 1   "red"    现在为红方
        int     temp_color_;     
        // 旋转矩阵        
        cv::Mat warp_matrix_;   

    private:
        cv::Mat getMatrix() {
            return this->warp_matrix_;
        }

    public:
        GetWarpMatrix(int click_point_method, int temp_color) {
            this->warp_matrix_          = cv::Mat(3, 3, CV_64FC1);
            this->click_point_method_   = click_point_method;
            this->temp_color_           = temp_color;
        }

        ~GetWarpMatrix() {}

    public:
        // 回调函数点击事件
        static void onMouse(int event, int x, int y, int, void* userInfo) {
            GetWarpMatrixInfo *info = (GetWarpMatrixInfo*)userInfo;

            static int          times   = 0;
            static cv::Point2f  four_point[4];
            if (event != cv::EVENT_LBUTTONDOWN) {
                return;
            }
            else {
                times++;
                if (times <= 4) {
                    std::cout << x << "  " << y << std::endl;
                    four_point[times-1].x = x;
                    four_point[times-1].y = y;
                }
                else if (times == 5) {
                    cv::Point2f god_view[4];

                    if (info->click_point_method_ == 0) {
                        if (info->temp_color_ == 0) {   // "blue"
                            god_view[0] = cv::Point2f(100, 808-638);
                            god_view[1] = cv::Point2f(348, 100);
                            god_view[2] = cv::Point2f(100, 708);
                            god_view[3] = cv::Point2f(348, 808-150);
                            // god_view[] 	= { cv::Point2f(100, 808-638), cv::Point2f(348, 100), cv::Point2f(100, 708), cv::Point2f(348, 808-150) };
                        }
                        if (info->temp_color_ == 1) {   // "red"
                            god_view[0] = cv::Point2f(100,      808-638);
                            god_view[1] = cv::Point2f(348,      100);
                            god_view[2] = cv::Point2f(100+20,   708);
                            god_view[3] = cv::Point2f(348,      808-150);
                            // god_view[] 	    = { cv::Point2f(100, 808-638), cv::Point2f(348, 100), cv::Point2f(100+20, 708), cv::Point2f(348, 808-150) };
                        }
                    }
                    else if (info->click_point_method_ == 1) {
                        god_view[0] = cv::Point2f(214,      808-578);
                        god_view[1] = cv::Point2f(93.5,     808-354);
                        god_view[2] = cv::Point2f(334.5,    808-354);
                        god_view[3] = cv::Point2f(214,      808-150);
                    }

                    // 计算变换矩阵
                    info->warp_matrix_ = cv::getPerspectiveTransform(four_point, god_view);
                    std::cout << "矩阵参数如下\n" << info->warp_matrix_ << std::endl;
                }
            }
        }

        // 获取透视变换矩阵
        cv::Mat getWarpMatrix(cv::Mat& img) {
            GetWarpMatrixInfo* userInfo = new GetWarpMatrixInfo(this->click_point_method_, this->temp_color_, this->warp_matrix_);
            
            while(true) {
                cv::imshow("click_point", img);
                cv::setMouseCallback("click_point", onMouse, userInfo); // 调用回调函数
                if (cv::waitKey(1) == 'q') {
                    cv::destroyWindow("click_point");
                    break;
                }
            }
            this->warp_matrix_ = userInfo->warp_matrix_;
            
            return getMatrix();
        }
};

// [0.8692728977430668, 2.614036593353056, -812.4954883898815;
//  -0.7891317045354738, 4.572244096696173, 338.5540557661475;
//  -0.0001577328925201915, 0.005119369921151915, 1]