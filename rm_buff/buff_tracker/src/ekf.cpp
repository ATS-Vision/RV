#include "buff_tracker/ekf.hpp"

namespace rm_buff
{
    EKF::EKF(
    const BuffVecVecFunc & f, const BuffVecVecFunc & h, const BuffVecMatFunc & j_f, const BuffVecMatFunc & j_h,
    const BuffVoidMatFunc & u_q, const BuffVecMatFunc & u_r, const Eigen::MatrixXd & P0)
    :f(f),h(h),
    jacobian_f(j_f),jacobian_h(j_h),
    update_Q(u_q),update_R(u_r),
    P_post(P0),n(P0.rows()),
    I(Eigen::MatrixXd::Identity(n, n)),
    x_pri(n),x_post(n)
    {}

    void EKF::setState(const Eigen::VectorXd & x0) {x_post = x0;}

    Eigen::MatrixXd EKF::predict()
    {
    // 计算状态转移函数的雅可比矩阵，并更新过程噪声协方差矩阵
    F = jacobian_f(x_post), Q = update_Q();
    //使用状态转移函数预测下一个状态
    x_pri = f(x_post);
    // 计算先验误差协方差矩阵
    P_pri = F * P_post * F.transpose() + Q;

    // 如果在下一个预测步骤之前没有测量，更新后验状态和误差协方差矩阵
    x_post = x_pri;
    P_post = P_pri;
    // 返回先验状态
    return x_pri;
    }

    Eigen::MatrixXd EKF::update(const Eigen::VectorXd & z)
    {
        // 计算观测函数的雅可比矩阵，并更新测量噪声协方差矩阵
        H = jacobian_h(x_pri), R = update_R(z);
        // 计算卡尔曼增益
        K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
        // 更新后验状态
        x_post = x_pri + K * (z - h(x_pri));
        // 更新后验误差协方差矩阵
        P_post = (I - K * H) * P_pri;
        // 返回后验状态
        return x_post;
    }
}