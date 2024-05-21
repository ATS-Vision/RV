#ifndef BUFF_DETECTOR_HPP_
#define BUFF_DETECTOR_HPP_

#include <iostream>
#include <future>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "buff_detector/param_struct.hpp"

namespace rm_buff
{
    struct Fan : ObjectBase
    {
        cv::Point2f apex2d[5];
        cv::Point2f center;
    };


}



#endif