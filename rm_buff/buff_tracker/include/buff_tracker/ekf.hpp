#ifndef BUFF_TRACKER_EKF_HPP_
#define BUFF_TRACKER_EKF_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_buff
{
    class EKF
    {
        public:
            EKF()=default;
            using BuffVecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
            using BuffVecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
            using BuffVoidMatFunc = std::function<Eigen::MatrixXd()>;
    
        explicit EKF(
        const BuffVecVecFunc & f, const BuffVecVecFunc & h, const BuffVecMatFunc & j_f, const BuffVecMatFunc & j_h,
        const BuffVoidMatFunc & u_q, const BuffVecMatFunc & u_r, const Eigen::MatrixXd & P0);

        // Set the initial state
        void setState(const Eigen::VectorXd & x0);
        // Compute a predicted state
        Eigen::MatrixXd predict();

        // Update the estimated state based on measurement
        Eigen::MatrixXd update(const Eigen::VectorXd & z);

    private:
        // 处理非线性向量函数
        BuffVecVecFunc f;
        // 观测非线性向量函数
        BuffVecVecFunc h;
        // f() 的雅可比行列式 
        BuffVecMatFunc jacobian_f;
        Eigen::MatrixXd F;
        //h() 的雅可比行列式 
        BuffVecMatFunc jacobian_h;
        Eigen::MatrixXd H;
        // 过程噪声协方差矩阵的更新函数
        BuffVoidMatFunc update_Q;
        Eigen::MatrixXd Q;
        // 观测噪声协方差矩阵的更新函数。
        BuffVecMatFunc update_R;
        Eigen::MatrixXd R;

        // 先验误差估计协方差矩阵，表示预测状态的误差协方差
        Eigen::MatrixXd P_pri;
    // 后验误差估计协方差矩阵，表示更新后状态的误差协方差
        Eigen::MatrixXd P_post;

        // 卡尔曼增益矩阵，用于平衡预测状态和观测值之间的差异。
        Eigen::MatrixXd K;

        //系统的维度，即状态向量的维数
        int n;

        // 维度为 `n` 的单位矩阵，用于矩阵计算中的单位操作
        Eigen::MatrixXd I;

        // 先验状态向量，表示预测的系统状态
        Eigen::VectorXd x_pri;
        // 后验状态向量，表示更新后的系统状态
        Eigen::VectorXd x_post;
    };
}

#endif