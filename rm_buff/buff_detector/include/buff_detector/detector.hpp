#ifndef BUFF_Detector_HPP_
#define BUFF_Detector_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include <Eigen/Dense>

#include <future>
#include <cmath>
#include <rclcpp/logger.hpp>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "buff_detector/window.hpp"
#include "buff_detector/param_struct.hpp"
#include "buff_detector/buff_tracker.hpp"
#include "buff_detector/pnp_solver.hpp"
// #include "buff_interfaces/msg/buff.hpp"
// #include "buff_interfaces/msg/buffs.hpp"

/**
 * 缺少可视化和主程序
*/

using namespace cv;

namespace rm_buff
{
    class Detector
    {

    public:
        Detector();
        ~Detector();
        Detector(const BuffParam& buff_param, const PathParam& path_param);
        std::vector<Fan> run(cv::Mat& src);
        cv::Point2i cropImageByROI(cv::Mat &img); //roi裁剪
    public:
        BuffParam buff_param_;
        PathParam path_param_;
        std::vector<Fan> fans;
        BuffDetector buff_detector_;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    private: 
        Size2d input_size_;
        int lost_cnt_;
        Point2i roi_offset_;
        Point2i last_roi_center_;
        rclcpp::Logger logger_;
    };
}



#endif