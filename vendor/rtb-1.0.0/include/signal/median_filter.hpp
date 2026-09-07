/**
 * @file median_filter.hpp
 * @brief 通用中值滤波器 (Median Filter)
 * @details
 * 原理: 维护一个滑动窗口，输出窗口内数据的中位数。
 * 特点:
 * 1. 能完美消除脉冲噪声 (Spikes)。
 * 2. 具有启动自适应功能 (首帧充满)。
 * 3. 实时性优化 (无运行时内存分配)。
 */

#ifndef RTB_CORE_MEDIAN_FILTER_HPP
#define RTB_CORE_MEDIAN_FILTER_HPP

#include <cstddef>
#include <vector>

namespace rtb {
namespace signal {

class MedianFilter {
public:
    /**
     * @brief 构造函数
     * @param window_size 窗口大小 (建议为奇数，如 3, 5, 7)，必须 > 0
     */
    explicit MedianFilter(int window_size);

    ~MedianFilter() = default;

    /**
     * @brief 重置滤波器
     * @details
     * 清除内部状态。下一次 update 时，会使用新输入填满整个窗口，
     * 以避免启动时的零值滞后 (Zero-lag initialization)。
     */
    auto reset() -> void ;

    /**
     * @brief 动态设置窗口大小
     * @warning 此函数涉及内存分配，禁止在实时循环中调用！请在初始化阶段调用。
     * @param window_size 新的窗口大小
     */
    auto setWindowSize(int window_size) -> void ;

    /**
     * @brief 执行滤波 (实时安全)
     * @param input 当前采样值
     * @return 滤波后的中值
     */
    auto update(double input) -> double ;

    /**
     * @brief 获取当前输出值
     */
    auto getValue() const -> double ;

private:
    int window_size_;
    int cursor_;          // 环形缓冲区写入指针
    bool initialized_;    // 初始化标志
    double prev_median_;  // 上一次的中值结果

    // 内存预分配 (Member variables to avoid allocation in loop)
    std::vector<double> buffer_;       // 环形数据缓冲区
    std::vector<double> sort_buffer_;  // 排序专用临时缓冲区 (避免栈上大数组或堆分配)
};

}  // namespace signal
}  // namespace rtb

#endif  // RTB_CORE_MEDIAN_FILTER_HPP