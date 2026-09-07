/**
 * @file high_pass_filter.hpp
 * @brief 通用一阶高通滤波器 (First-Order High Pass Filter)
 * @details
 * 传递函数: H(s) = (tau * s) / (tau * s + 1)
 * 差分方程: y[k] = alpha * (y[k-1] + x[k] - x[k-1])
 * 适用于实时系统，无动态内存分配。
 */

#ifndef RTB_CORE_HIGH_PASS_FILTER_HPP
#define RTB_CORE_HIGH_PASS_FILTER_HPP

namespace rtb {
namespace signal {

class HighPassFilter {
public:
    /**
     * @brief 构造函数
     * @param cutoff_freq_hz 截止频率 (Hz)，必须 > 0
     * @param sample_time_s  采样周期 (秒)，必须 > 0
     * @param initial_input  初始输入值 (用于差分计算，默认为0)
     * @param initial_output 初始输出值 (默认为0)
     */
    explicit HighPassFilter(
        double cutoff_freq_hz, double sample_time_s, double initial_input = 0.0, double initial_output = 0.0);

    ~HighPassFilter() = default;

    /**
     * @brief 重置滤波器状态
     */
    auto reset(double initial_input = 0.0, double initial_output = 0.0) -> void ;

    /**
     * @brief 动态更新参数
     */
    auto setParameters(double cutoff_freq_hz, double sample_time_s) -> void ;

    /**
     * @brief 执行滤波计算 (实时调用)
     * @param input 当前采样时刻的原始输入值
     * @return 滤波后的输出值 (保留高频，滤除直流/低频)
     */
    auto update(double input) -> double ;

    /**
     * @brief 获取当前输出值
     */
    auto getValue() const -> double ;

private:
    auto computeAlpha() -> void ;

private:
    double cutoff_freq_;  // Hz
    double dt_;           // s
    double alpha_;        // 滤波系数

    double prev_input_;   // 上一时刻输入 x[k-1]
    double prev_output_;  // 上一时刻输出 y[k-1]

    static constexpr double EPSILON = 1e-9;
};

}  // namespace signal
}  // namespace rtb

#endif  // RTB_CORE_HIGH_PASS_FILTER_HPP