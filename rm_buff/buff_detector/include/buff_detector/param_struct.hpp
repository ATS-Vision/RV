#ifndef BUFF_PARAM_STRUCT_HPP_
#define BUFF_PARAM_STRUCT_HPP_

#include <future>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>

#include <opencv2/opencv.hpp>

namespace rm_buff
{
    const int RED = 0;
    const int BLUE = 1;
    
    enum BuffStatus
    {
        UNACTIVATED,
        ACTIVATED
    };
    
    struct Object
    {
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    struct ObjectBase
    {
        int id;
        int color;
        double conf;
        std::string key;
    
    };

        struct BuffParam
    {
        int color;
        // int max_lost_cnt;           // 最大丢失目标帧数
        // double max_v;                  // 最大旋转速度(rad/s)
        // double max_delta_t;            // 使用同一预测器的最大时间间隔(ms)
        // double fan_length;          // 大符臂长(R字中心至装甲板中心)
        // double no_crop_thres;       // 禁用ROI裁剪的装甲板占图像面积最大面积比值
        // double max_angle;

        BuffParam()
        {
            color = 1;
            // max_lost_cnt = 10; 
            // max_v = 4.0;   
            // max_delta_t = 200; 
            // fan_length = 0.7;
            // no_crop_thres = 2e-3;
            // max_angle = 0.25;
        }
    };

    struct PathParam
    {
    //    std::string camera_name;
        std::string network_path;
    //    std::string camera_param_path;
    //    std::string path_prefix;

        PathParam()
        {
     //       camera_name = "KE0200110075";
            network_path = "../model/buff.xml";
     //       camera_param_path = "src/global_user/config/camera.yaml";
      //      path_prefix = "src/recorder/buff_dataset";
        }
    };


    
}











#endif