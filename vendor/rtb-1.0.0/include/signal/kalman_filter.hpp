/**
 * @file kalman_filter.hpp
 * @brief 通用离散卡尔曼滤波器 (Generic Discrete Kalman Filter)
 * @details
 * 实现标准的线性卡尔曼滤波：
 * 1. 预测: x = Fx + Bu, P = FPF' + Q
 * 2. 更新: K = PH'(HPH' + R)^-1, x = x + K(z - Hx), P = (I - KH)P
 * * 特点：
 * - 实时性优化：所有内存（包括中间临时变量）在 init 阶段一次性分配。
 * - 通用性：支持运行时配置状态维数。
 */

#ifndef RTB_SIGNAL_KALMAN_FILTER_HPP
#define RTB_SIGNAL_KALMAN_FILTER_HPP

#include <cstddef>
#include <vector>

namespace rtb {
namespace signal {

class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();

    /**
     * @brief 初始化滤波器维度并分配内存
     * @param state_dim 状态向量维度 (x)
     * @param measure_dim 观测向量维度 (z)
     * @param control_dim 控制向量维度 (u)，如果没有控制输入设为0
     */
    auto init(int state_dim, int measure_dim, int control_dim = 0) -> void ;

    /**
     * @brief 重置状态和协方差
     * @details F, H, Q, R 等参数矩阵保持不变
     */
    auto resetState(const std::vector<double>& x0, const std::vector<double>& P0) -> void ;

    /**
     * @brief 执行预测步骤 (Time Propagation)
     * @param u 控制输入向量 (若 control_dim=0，可传空 vector)
     */
    auto predict(const std::vector<double>& u = {}) -> void ;

    /**
     * @brief 执行更新/校正步骤 (Measurement Update)
     * @param z 测量值向量
     * @return 状态更新是否成功 (如果矩阵不可逆返回 false)
     */
    auto update(const std::vector<double>& z) -> bool ;

    // ================== 参数设置接口 ==================
    // 为了高效，这里提供直接指针访问或 vector set 接口

    // 设置状态转移矩阵 F (n x n)
    auto setF(const std::vector<double>& F) -> void ;
    // 设置观测矩阵 H (m x n)
    auto setH(const std::vector<double>& H) -> void ;
    // 设置过程噪声协方差 Q (n x n)
    auto setQ(const std::vector<double>& Q) -> void ;
    // 设置测量噪声协方差 R (m x m)
    auto setR(const std::vector<double>& R) -> void ;
    // 设置控制矩阵 B (n x c)
    auto setB(const std::vector<double>& B) -> void ;
    // 设置当前状态 x (n x 1)
    auto setX(const std::vector<double>& x) -> void ;
    // 设置当前协方差 P (n x n)
    auto setP(const std::vector<double>& P) -> void ;

    // ================== 获取结果 ==================
    // 获取当前状态估计 x
    auto getState() const -> std::vector<double> { return x_; }

    // 获取当前协方差 P
    auto getCovariance() const -> std::vector<double> { return P_; }

    // 获取特定状态分量 (便捷函数)
    auto getState(int idx) const -> double ;

private:
    // 维度记录
    std::size_t n_;  // State dim
    std::size_t m_;  // Measure dim
    std::size_t c_;  // Control dim
    bool initialized_;

    // 核心矩阵 (使用 vector 存储，行优先展开)
    std::vector<double> x_;  // State vector (n x 1)
    std::vector<double> P_;  // Covariance matrix (n x n)

    std::vector<double> F_;  // State transition (n x n)
    std::vector<double> H_;  // Measurement matrix (m x n)
    std::vector<double> Q_;  // Process noise (n x n)
    std::vector<double> R_;  // Measurement noise (m x m)
    std::vector<double> B_;  // Control matrix (n x c)
    std::vector<double> K_;  // Kalman Gain (n x m)

    // ==========================================
    // 预分配的临时变量 (Pre-allocated buffers)
    // 避免在 update 循环中进行 new/malloc
    // ==========================================

    // 预测步骤临时变量
    std::vector<double> tmp_nx1_;    // For F*x
    std::vector<double> tmp_nxn_;    // For F*P
    std::vector<double> tmp_nxn_2_;  // For (F*P)*F'
    std::vector<double> tmp_nxn_3_;  // For P calculation
    std::vector<double> F_t_;        // F transpose

    // 更新步骤临时变量
    std::vector<double> y_;               // Innovation (m x 1)
    std::vector<double> S_;               // Innovation covariance (m x m)
    std::vector<double> S_inv_;           // Inverse of S (m x m)
    std::vector<double> H_t_;             // H transpose (n x m)
    std::vector<double> P_Ht_;            // P * H' (n x m)
    std::vector<double> tmp_mxn_;         // H * P
    std::vector<double> tmp_mxm_;         // H * P * H'
    std::vector<double> I_;               // Identity matrix (n x n)
    std::vector<double> tmp_nxn_update_;  // For (I - KH)P
};

}  // namespace signal
}  // namespace rtb

#endif  // RTB_SIGNAL_KALMAN_FILTER_HPP