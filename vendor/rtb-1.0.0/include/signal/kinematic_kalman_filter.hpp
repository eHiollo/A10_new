#pragma once

#include <memory>

namespace rtb {
namespace signal {

/**
 * @brief 多轴运动学卡尔曼滤波器封装类
 * @details 底层复用 rtb::signal::KalmanFilter，实现 N 个解耦的 3 阶恒加速系统模型。
 */
class KinematicKalmanFilter {
public:
    KinematicKalmanFilter();
    ~KinematicKalmanFilter();

    KinematicKalmanFilter(const KinematicKalmanFilter&) = delete;
    KinematicKalmanFilter& operator=(const KinematicKalmanFilter&) = delete;
    KinematicKalmanFilter(KinematicKalmanFilter&&) noexcept;
    KinematicKalmanFilter& operator=(KinematicKalmanFilter&&) noexcept;

    auto setInputSize(int input_size) -> void ;
    auto setDt(double dt) -> void ;

    /**
     * @brief 设置噪声协方差
     * @param R  位置测量的噪声方差 (如: 编码器量化误差平方)
     * @param Qa 加速度过程噪声方差 (代表实际运动违背"恒定加速度模型"的剧烈程度)
     */
    auto setNoiseCovariance(double R, double Qa) -> void ;

    auto init(const double* q_init) -> void ;

    auto update(const double* q_measure, double* q_hat, double* v_hat, double* a_hat) -> void ;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace signal
}  // namespace rtb