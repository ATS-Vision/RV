#ifndef BUFF_PNP_SOLVER_H_
#define BUFF_PNP_SOLVER_H_
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core/eigen.hpp>
#include <array>
#include <opencv2/core/types.hpp>
#include <vector>

#include "buff_detector/buff_tracker.hpp"

namespace rm_buff
{
    class PnPSolver
    {
        public:
        PnPSolver();
        ~PnPSolver();
        PnPSolver(
            const std::array<double, 9> & camera_matrix,
            const std::vector<double> & distortion_coefficients);

        // Get 3d position
        bool solvePnP(const Fan & fan, cv::Mat & rvec, cv::Mat & tvec);

        // Calculate the distance between armor center and image center
        float calculateDistanceToCenter(const cv::Point2f & image_point);
        cv::Point2f reproject(Eigen::Vector3d &xyz);

    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // std::vector<cv::Point3f> fan_world_;
    };


}

#endif