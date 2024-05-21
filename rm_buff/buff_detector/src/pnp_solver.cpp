#include "buff_detector/pnp_solver.hpp"
#include <opencv2/core/types.hpp>

namespace rm_buff
{
    PnPSolver::PnPSolver() {}
    PnPSolver::~PnPSolver() {}
    PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
}

bool  PnPSolver::solvePnP(const Fan & fan, cv::Mat & rvec, cv::Mat & tvec)
{
    std::vector<cv::Point3f> fan_world_;
    fan_world_= 
            {
                {0, -0.7, -0.05},
                {-0.11, -0.11, 0.0},
                {-0.11, 0.11, 0.0},
                {0.11, 0.11, 0.0},
                {0.11, -0.11, 0.0},
            };
    std::vector<cv::Point2f> points_pic(fan.apex2d, fan.apex2d + 5);
    return cv::solvePnP(fan_world_, points_pic, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_EPNP)
            && cv::solvePnP(fan_world_, points_pic, camera_matrix_, dist_coeffs_, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

cv::Point2f PnPSolver::reproject(Eigen::Vector3d &xyz)
{
  Eigen::Matrix3d mat_intrinsic;
  cv::cv2eigen(camera_matrix_, mat_intrinsic);
  //(u,v,1)^T = (1/Z) * K * (X,Y,Z)^T
  auto result = (1.f / xyz[2]) * mat_intrinsic * (xyz);//解算前进行单位转换
  return cv::Point2f(result[0], result[1]);
}

}