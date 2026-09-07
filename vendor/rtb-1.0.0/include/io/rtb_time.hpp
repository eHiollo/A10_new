/**
 * @file rtb_time.hpp
 * @brief RTB 高精度时间库
 * @details 提供单调递增的系统时间，用于控制回路和日志记录
 */
#pragma once

#include <chrono>
#include <thread>

namespace rtb {
namespace io {

// ============================================================================
// 核心时间函数
// ============================================================================

/**
 * @brief 获取当前单调时间（秒，double精度）
 * @details
 * 使用 steady_clock，不受系统时间修改影响。
 * 此时钟通常从系统启动开始计时，或者从库初始化开始计时。
 * 精度：通常为微秒或纳秒级，但以 double 秒返回。
 * 适合：PID 计算、卡尔曼滤波、物理模拟步长。
 */
inline auto now() -> double {
    // 静态变量只初始化一次，作为基准时间（如果希望获取程序运行时间）
    // 如果希望获取系统 uptime，可以去掉 start_time 减法
    static auto start_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();

    // 返回从程序启动开始经过的秒数
    return std::chrono::duration<double>(current_time - start_time).count();
}

/**
 * @brief 获取系统绝对时间戳（秒，double精度）
 * @details
 * 对应墙上时间（Wall Clock），即 1970年1月1日以来的秒数。
 * 警告：此时间可能会跳变！
 * 适合：日志记录中的日期显示、与外部系统（如数据库）同步。
 */
inline auto wall_time() -> double {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

/**
 * @brief 获取当前单调时间（纳秒，uint64_t）
 * @details 适合极致性能分析或需要整数时间戳的场景
 */
inline auto now_ns() -> uint64_t {
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

// ============================================================================
// 频率控制辅助类
// ============================================================================
/**
 * @brief 简单的循环频率控制器 (Rate Keeper)
 * @example
 * rtb::time::Rate rate(100); // 100Hz
 * while(running) {
 * // do control...
 * rate.sleep(); // 自动休眠剩余时间
 * }
 */
class Rate {
public:
    explicit Rate(double frequency_hz)
        : period_(1.0 / frequency_hz),
          last_time_(std::chrono::steady_clock::now()) {}

    auto sleep() -> void {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_time_).count();
        auto sleep_sec = period_ - elapsed;

        if (sleep_sec > 0) {
            // 注意：sleep_for 在非实时系统上可能有 1-2ms 的抖动
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_sec));
        }

        // 更新 last_time_，为了防止时间漂移，应该加上固定的周期
        // 但简单实现重置为当前时间即可
        last_time_ = std::chrono::steady_clock::now();
    }

    // 重置起始点
    auto reset() -> void { last_time_ = std::chrono::steady_clock::now(); }

private:
    double period_;
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace io
}  // namespace rtb