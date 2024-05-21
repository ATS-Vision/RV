#include "buff_detector/detector.hpp"
#include "buff_detector/buff_tracker.hpp"
#include "buff_detector/param_struct.hpp"
#include <complex>
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <vector>

namespace rm_buff
{
    Detector::Detector()
    : logger_(rclcpp::get_logger("buff_detector"))
    {
        // lost_cnt_ = 0;
        // is_last_target_exists_ = false;
        // input_size_ = {640,384};
        input_size_ = {640, 640};
        // last_bullet_speed_ = 0;
        // last_angle_ = 0.0;
    }

    Detector::Detector(const BuffParam& buff_param, const PathParam& path_param)
    :buff_param_(buff_param),path_param_(path_param),logger_(rclcpp::get_logger("buff_detector"))
    {
        input_size_ = {640, 640};
    }

    Detector::~Detector() {}

    cv::Point2i Detector::cropImageByROI(cv::Mat &img)
    {
        // if (lost_cnt_ > buff_param_.max_lost_cnt || lost_cnt_ == 0)
        // {
        //     return Point2i(0,0);
        // }
        
        // 处理X越界
        if (last_roi_center_.x <= input_size_.width / 2)
            last_roi_center_.x = input_size_.width / 2;
        else if (last_roi_center_.x > (img.size().width - input_size_.width / 2))
            last_roi_center_.x = img.size().width - input_size_.width / 2;
        // 处理Y越界
        if (last_roi_center_.y <= input_size_.height / 2)
            last_roi_center_.y = input_size_.height / 2;
        else if (last_roi_center_.y > (img.size().height - input_size_.height / 2))
            last_roi_center_.y = img.size().height - input_size_.height / 2;

        // 左上角顶点
        auto offset = last_roi_center_ - Point2i(input_size_.width / 2, input_size_.height / 2);
        Rect roi_rect = Rect(offset, input_size_);
        img(roi_rect).copyTo(img);

        return offset;
    }

    std::vector<Fan> Detector::run(cv::Mat &src)
    {
        
        auto time_start = steady_clock_.now();
        std::vector<BuffObject> objects;
        auto input=src;
        // roi_offset_ = cropImageByROI(input);
        fans.clear();
        if (!buff_detector_.detect(input, objects))
        {   //若未检测到目标
            lost_cnt_++;
            // is_last_target_exists_ = false;
            // last_target_area_ = 0;
            last_roi_center_={320,320};
            RCLCPP_WARN(logger_, "No buff target is detected...");
            
        }else
        {
            auto time_infer = steady_clock_.now();
            roi_offset_ = cropImageByROI(input);
            for (auto object : objects)
            {
                if (buff_param_.color == RED)
                {
                    if (object.color != RED)
                        continue;
                }
                else if (buff_param_.color == BLUE)
                {
                    if (object.color != BLUE)
                        continue;
                }
                Fan fan;
                fan.id = object.cls;
                fan.color = object.color;
                fan.conf = object.prob;
                if (object.color == 0)
                {
                    fan.key = "B" + std::string(object.cls == UNACTIVATED ? "Target" : "Activated");
                }
                else if (object.color == 1)
                {
                    fan.key = "R" + std::string(object.cls == UNACTIVATED ? "Target" : "Activated");
                }
                // memcpy(fan.apex2d, object.apex, 5 * sizeof(cv::Point2f));
                for (int i = 0; i < 5; i++)
                {
                    fan.apex2d[i] += Point2f((float)roi_offset_.x, (float)roi_offset_.y);
                }
                last_roi_center_={int(fan.apex2d[0].x),int(fan.apex2d[0].y)};
                std::vector<Point2f> points_pic(fan.apex2d, fan.apex2d + 5);
                fan.center={abs((fan.apex2d[2].x-fan.apex2d[3].x)/2),abs((fan.apex2d[2].y-fan.apex2d[1].y)/2)};
                fans.emplace_back(fan);
            }
        }
    return fans;
}


}