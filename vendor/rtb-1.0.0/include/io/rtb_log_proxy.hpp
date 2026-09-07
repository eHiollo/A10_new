/**
 * @file rtb_log_proxy.hpp
 * @brief 模块化日志代理：根据 RTB_MODULE_TAG 自动判断是否开启日志
 * @date 2025-01
 *
 * @details
 * 本文件实现了模块化的日志系统，支持：
 * 1. 在 CMake 中通过 option 控制每个模块的日志开关
 * 2. 在 C++ 源码中极简使用，无需手写 #ifdef
 * 3. 编译期完全消除，零性能损耗
 * 4. 支持流式和函数式两种写法
 *
 * 使用示例：
 * @code
 * // 在 .cpp 文件开头
 * #define RTB_MODULE_TAG SRS
 * #include "io/rtb_log_proxy.hpp"
 *
 * // 使用流式输出
 * LOG_INFO_S << "Position: " << pos;
 *
 * // 使用函数式输出
 * LOG_INFO("Position: [{:.2f}, {:.2f}, {:.2f}]", x, y, z);
 * @endcode
 */

#pragma once

// ============================================================================
// 预处理宏：用于拼接宏名称
// ============================================================================
#define RTB_JOIN_IMPL(a, b) a##b
#define RTB_JOIN(a, b) RTB_JOIN_IMPL(a, b)

// 将宏的值转为字符串（用于生成 logger name）
#define RTB_STR_IMPL(x) #x
#define RTB_STR(x) RTB_STR_IMPL(x)

// ============================================================================
// 检查必要宏
// ============================================================================
#ifndef RTB_MODULE_TAG
#error "Error: You must '#define RTB_MODULE_TAG XXX' before including rtb_log_proxy.hpp"
#endif

// ============================================================================
// 自动判定开关
// 原理：拼接 RTB_LOG_ + TAG + _ON
// 如果 CMake 定义了 RTB_LOG_SRS_ON，那么就会展开为 1
// 如果没定义，预处理器会将其视为 0（在 #if 判断中）
// ============================================================================
#define RTB_CURRENT_SWITCH_MACRO RTB_JOIN(RTB_JOIN(RTB_LOG_, RTB_MODULE_TAG), _ON)

// ============================================================================
// 分支实现
// ============================================================================
#if defined(RTB_CURRENT_SWITCH_MACRO) && defined(RTB_LOG_ENABLED)
// 模块日志已开启且全局日志已开启

// 引入真正的实现
#include "io/rtb_logger.hpp"

// 自动推导 Logger Name，如果用户没自定义 RTB_MODULE_NAME，就用 TAG 转字符串
#ifndef RTB_MODULE_NAME
#define RTB_MODULE_NAME RTB_STR(RTB_MODULE_TAG)
#endif

// ========================================================================
// 开启状态下的宏定义
// ========================================================================

// A. 流式输出 (Stream) - 返回 LogStream 对象
// 注意：RTB_LOG_XXX 宏支持流式输出，直接调用即可
#define LOG_DEBUG_S RTB_LOG_DEBUG(RTB_MODULE_NAME)
#define LOG_INFO_S RTB_LOG_INFO(RTB_MODULE_NAME)
#define LOG_WARN_S RTB_LOG_WARN(RTB_MODULE_NAME)
#define LOG_ERROR_S RTB_LOG_ERROR(RTB_MODULE_NAME)

// B. 函数式输出 (Format) - 使用 RTB_LOG_XXX 宏
#define LOG_DEBUG(...) RTB_LOG_DEBUG(RTB_MODULE_NAME, __VA_ARGS__)
#define LOG_INFO(...) RTB_LOG_INFO(RTB_MODULE_NAME, __VA_ARGS__)
#define LOG_WARN(...) RTB_LOG_WARN(RTB_MODULE_NAME, __VA_ARGS__)
#define LOG_ERROR(...) RTB_LOG_ERROR(RTB_MODULE_NAME, __VA_ARGS__)

#else
// ========================================================================
// 关闭状态（编译期擦除）
// ========================================================================

// 引入 NullStream 定义（如果还没引入）
#ifndef RTB_LOGGER_HPP
#include <iostream>

namespace rtb {
namespace io {
class NullStream {
public:
    template <typename T>
    NullStream& operator<<(const T&) { return *this; }

    NullStream& operator<<(std::ostream& (*)(std::ostream&)) { return *this; }
};
}  // namespace io
}  // namespace rtb
#else
// 如果 rtb_logger.hpp 已包含，直接使用其 NullStream
// 不需要再次包含
#endif

// 流式输出：返回 NullStream 对象（编译期优化后为零开销）
#define LOG_DEBUG_S rtb::io::NullStream()
#define LOG_INFO_S rtb::io::NullStream()
#define LOG_WARN_S rtb::io::NullStream()
#define LOG_ERROR_S rtb::io::NullStream()

// 函数式输出：展开为 do {} while(0)，编译期完全消除
#define LOG_DEBUG(...) \
    do {               \
    } while (0)
#define LOG_INFO(...) \
    do {              \
    } while (0)
#define LOG_WARN(...) \
    do {              \
    } while (0)
#define LOG_ERROR(...) \
    do {               \
    } while (0)

#endif

// ============================================================================
// 清理临时宏（避免污染命名空间）
// ============================================================================
#undef RTB_CURRENT_SWITCH_MACRO
