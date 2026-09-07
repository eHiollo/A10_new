/**
 * @file low_pass_filter.hpp
 * @brief 通用一阶低通滤波器 (First-Order Low Pass Filter)
 * @details
 * 基于离散化的一阶惯性环节实现: Y(s)/X(s) = 1 / (tau*s + 1)
 * 差分方程: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 * 适用于实时控制系统，无动态内存分配。
 */

#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP

namespace rtb {
namespace signal {

class LowPassFilter {
public:
    /**
     * @brief 构造函数
     * @param cutoff_freq_hz 截止频率 (Hz)，必须 > 0
     * @param sample_time_s  采样周期 (秒)，必须 > 0
     * @param initial_value  初始值 (默认为0)
     */
    explicit LowPassFilter(double cutoff_freq_hz, double sample_time_s, double initial_value = 0.0);

    /**
     * @brief 析构函数
     */
    ~LowPassFilter() = default;

    /**
     * @brief 初始化/重置滤波器状态
     * @param value 重置后的输出值
     */
    auto reset(double value = 0.0) -> void ;

    /**
     * @brief 更新滤波器参数
     * @details 允许在运行时动态修改截止频率或采样时间
     * @param cutoff_freq_hz 新的截止频率 (Hz)
     * @param sample_time_s  新的采样周期 (s)
     */
    auto setParameters(double cutoff_freq_hz, double sample_time_s) -> void ;

    /**
     * @brief 执行滤波计算 (核心实时函数)
     * @param input 当前采样时刻的原始输入值
     * @return 滤波后的输出值
     */
    auto update(double input) -> double ;

    /**
     * @brief 获取当前滤波器的输出值 (不进行更新)
     */
    auto getValue() const -> double ;

private:
    /**
     * @brief 重新计算滤波系数 Alpha
     */
    auto computeAlpha() -> void ;

private:
    double cutoff_freq_;  // 截止频率 Hz
    double dt_;           // 采样周期 s
    double alpha_;        // 滤波系数 [0, 1]
    double prev_output_;  // 上一时刻的输出值 y[k-1]

    // 常量定义
    static constexpr double EPSILON = 1e-6;
};

}  // namespace signal
}  // namespace rtb

#endif  // LOW_PASS_FILTER_HPP