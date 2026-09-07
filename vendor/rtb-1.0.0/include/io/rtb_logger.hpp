/**
 * @file rtb_logger.hpp
 * @brief RTB 日志系统 - 统一的日志接口和 spdlog 便捷封装
 * @date 2025-01
 *
 * @details
 * 本文件提供了 RTB 库的完整日志解决方案：
 * 1. LoggerManager - 线程安全的多个logger管理
 * 2. 统一日志接口 - 支持流式输出和格式化字符串（C++23风格）
 * 3. 模块级条件编译支持
 * 4. 全局开关支持（RTB_LOG_DISABLED）
 *
 * 使用示例：
 * @code
 * // 初始化
 * rtb::io::init_async();
 * rtb::io::add_file_logger("motion", "log/motion.log");
 *
 * // 流式输出
 * RTB_LOG_INFO("motion") << "Position: " << pos;
 *
 * // 格式化字符串（C++23风格）
 * RTB_LOG_INFO("motion", "Position: [{:.2f}, {:.2f}, {:.2f}]", x, y, z);
 * @endcode
 */

#ifndef RTB_LOGGER_HPP
#define RTB_LOGGER_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>

// ============================================================================
// 全局开关检查（最高优先级）
// ============================================================================
#ifdef RTB_LOG_DISABLED
// 如果定义了 RTB_LOG_DISABLED，所有日志功能完全关闭
// 所有宏展开为 ((void)0)，编译期完全消除
#define RTB_LOG_DEBUG(name, ...) ((void)0)
#define RTB_LOG_INFO(name, ...) ((void)0)
#define RTB_LOG_WARN(name, ...) ((void)0)
#define RTB_LOG_ERROR(name, ...) ((void)0)
#define RTB_LOG_DEBUG_FMT(name, ...) ((void)0)
#define RTB_LOG_INFO_FMT(name, ...) ((void)0)
#define RTB_LOG_WARN_FMT(name, ...) ((void)0)
#define RTB_LOG_ERROR_FMT(name, ...) ((void)0)
// JSON 相关宏也完全消除
#define RTB_PLOT(logger_name, ...) ((void)0)
#define RTB_JSON(logger_name, ...) ((void)0)
#else

// ============================================================================
// NullStream 类定义（用于禁用日志时的流式输出兼容）
// ============================================================================
namespace rtb {
namespace io {

// 前向声明
class MatrixFormatter;

// 空操作流类：支持 << 操作符但不产生任何输出
// 当 RTB_LOG_ENABLED 未定义时，用于提供流式日志的兼容性
class NullStream {
public:
    template <typename T>
    NullStream& operator<<(const T&) { return *this; }

    NullStream& operator<<(std::ostream& (*)(std::ostream&)) { return *this; }

    // 支持 MatrixFormatter（虽然什么都不做）
    NullStream& operator<<(const MatrixFormatter&) { return *this; }
};

}  // namespace io
}  // namespace rtb

// ============================================================================
// RTB_LOG_ENABLED 检查
// ============================================================================
#ifdef RTB_LOG_ENABLED
// 定义 FMT_HEADER_ONLY 以启用 header-only 模式
#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include "io/spdlog/fmt/bundled/format.h"
#endif
#endif

namespace rtb {
namespace io {

// ============================================================================
// 前向声明和类型定义
// ============================================================================
#ifdef RTB_LOG_ENABLED
// 输出位置枚举
enum class LogOutput {
    Console,  // 仅终端
    File,     // 仅文件
    Both      // 终端+文件
};

// 日志级别枚举（隐藏 spdlog 的具体类型）
enum class LogLevel {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3
};

// Logger配置结构
struct LoggerConfig {
    std::string name;
    std::string pattern;    // spdlog格式字符串，如 "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v"
    LogOutput output;       // 输出位置
    std::string file_path;  // 文件路径（如果输出到文件）
    size_t max_file_size;   // 文件大小限制（轮转）
    size_t max_files;       // 保留文件数量

    LoggerConfig() : pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v"),
                     output(LogOutput::File),
                     max_file_size(SIZE_MAX - 1024 * 1024 * 1024),  // 避免溢出
                     max_files(5) {}
};

// Logger句柄（隐藏 spdlog::logger 的具体类型）
// 注意：由于模板函数需要访问 LoggerHandle 的方法，我们需要在头文件中声明接口
// 实际实现在 .cpp 文件中
class LoggerHandle {
public:
    auto log(LogLevel level, const std::string& message) -> void ;
    auto flush() -> void ;

    // 格式化日志方法（模板方法，必须在头文件中实现）
    // 注意：spdlog 的 logger->log 已经支持格式化字符串，不需要手动调用 fmt::format
    // 由于 Impl 是不完整类型，我们使用辅助函数来实现
    template <typename... Args>
    auto log_fmt(LogLevel level, Args&&... args) -> void {
        logger_handle_log_fmt_impl(this, level, std::forward<Args>(args)...);
    }

private:
    // 实现细节在 .cpp 文件中
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    // 构造函数在 .cpp 中实现
    LoggerHandle();

    // 前向声明 LoggerManager::Impl
    friend class LoggerManager;

    // 允许辅助函数访问 pimpl_
    template <typename... Args>
    friend auto logger_handle_log_fmt_impl(LoggerHandle* handle, LogLevel level, Args&&... args) -> void ;
};

// ========================================================================
// LoggerManager 单例类（线程安全，PIMPL模式）
// ========================================================================
class LoggerManager {
public:
    static auto instance() -> LoggerManager& {
        static LoggerManager inst;
        return inst;
    }

    // 禁止拷贝和移动
    LoggerManager(const LoggerManager&) = delete;
    LoggerManager& operator=(const LoggerManager&) = delete;
    LoggerManager(LoggerManager&&) = delete;
    LoggerManager& operator=(LoggerManager&&) = delete;

    // 添加logger
    auto add_logger(const LoggerConfig& config) -> bool ;
    bool add_logger(const std::string& name,
        const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
        LogOutput output = LogOutput::File,
        const std::string& file_path = "",
        size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
        size_t max_files = 5);                                 // 保留文件数量

    // 便捷函数：创建终端logger
    auto add_console_logger(const std::string& name,
        const std::string& pattern = "[%l] %v") -> bool ;

    // 便捷函数：创建文件logger
    bool add_file_logger(const std::string& name,
        const std::string& file_path,
        const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
        size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
        size_t max_files = 5);                                 // 保留文件数量

    // 便捷函数：创建终端+文件logger
    bool add_console_file_logger(const std::string& name,
        const std::string& file_path,
        const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
        size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
        size_t max_files = 5);                                 // 保留文件数量

    // 便捷函数：创建UDP logger（用于实时数据可视化）
    auto add_udp_logger(const std::string& name,
        const std::string& host = "127.0.0.1",
        uint16_t port = 9870) -> bool ;

    /**
     * @brief 添加 Graylog 专用日志器 (GELF格式 over UDP)
     * @param name Logger名称
     * @param host Graylog 服务器 IP
     * @param port Graylog GELF UDP 端口 (默认 12201)
     * @param source_host 发送方主机标识 (显示在 Graylog source 字段)
     */
    auto add_graylog_logger(const std::string& name,
        const std::string& host,
        uint16_t port = 12201,
        const std::string& source_host = "rtb-node") -> bool ;

    // 获取logger（返回不透明句柄）
    auto get_logger(const std::string& name) -> std::shared_ptr<LoggerHandle> ;

    // 更新logger配置
    auto update_logger_pattern(const std::string& name, const std::string& pattern) -> bool ;
    auto update_logger_output(const std::string& name, LogOutput output) -> bool ;

    // 移除logger
    auto remove_logger(const std::string& name) -> bool ;

    // 工具函数
    auto flush_all() -> void ;
    auto shutdown() -> void ;

    ~LoggerManager();

private:
    LoggerManager();

    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

// ========================================================================
// 初始化函数
// ========================================================================
auto init_async(size_t queue_size = 8192, size_t thread_count = 1) -> void ;
auto init_sync() -> void ;

// ========================================================================
// 全局函数包装（便捷接口）
// ========================================================================
inline auto add_console_logger(const std::string& name,
    const std::string& pattern = "[%l] %v") -> bool {
    return LoggerManager::instance().add_console_logger(name, pattern);
}

inline bool add_file_logger(const std::string& name,
    const std::string& file_path,
    const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
    size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
    size_t max_files = 5) {                                // 保留文件数量
    return LoggerManager::instance().add_file_logger(name, file_path, pattern, max_file_size, max_files);
}

inline bool add_console_file_logger(const std::string& name,
    const std::string& file_path,
    const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
    size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
    size_t max_files = 5) {                                // 保留文件数量
    return LoggerManager::instance().add_console_file_logger(name, file_path, pattern, max_file_size, max_files);
}

inline auto add_udp_logger(const std::string& name,
    const std::string& host = "127.0.0.1",
    uint16_t port = 9870) -> bool {
    return LoggerManager::instance().add_udp_logger(name, host, port);
}

inline auto add_graylog_logger(const std::string& name,
    const std::string& host,
    uint16_t port = 12201,
    const std::string& source_host = "rtb-node") -> bool {
    return LoggerManager::instance().add_graylog_logger(name, host, port, source_host);
}

inline auto update_logger_pattern(const std::string& name, const std::string& pattern) -> bool {
    return LoggerManager::instance().update_logger_pattern(name, pattern);
}

inline auto get_logger(const std::string& name) -> std::shared_ptr<LoggerHandle> {
    return LoggerManager::instance().get_logger(name);
}

inline auto remove_logger(const std::string& name) -> bool {
    return LoggerManager::instance().remove_logger(name);
}

inline auto flush_all() -> void {
    LoggerManager::instance().flush_all();
}

inline auto shutdown() -> void {
    LoggerManager::instance().shutdown();
}

// ========================================================================
// 统一日志接口 - 流式输出（部分PIMPL模式）
// ========================================================================
// 注意：为了支持模板方法，stream 成员在头文件中
// 只有 logger 和 level 在 Impl 中（隐藏 spdlog 类型）
class LogStream {
public:
    LogStream(std::shared_ptr<LoggerHandle> logger, LogLevel level);
    ~LogStream();

    // 禁止拷贝
    LogStream(const LogStream&) = delete;
    LogStream& operator=(const LogStream&) = delete;

    // 支持移动（使用 default，让编译器自动推断异常规范）
    LogStream(LogStream&&) = default;
    LogStream& operator=(LogStream&&) = default;

    // 流式输出操作符（模板方法，必须在头文件中实现）
    template <typename T>
    LogStream& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

    // 支持 std::endl, std::flush 等
    LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        manip(stream_);
        return *this;
    }

    // 专门的重载：支持 MatrixFormatter（在文件末尾实现）
    // 注意：这个重载必须在模板版本之后，以确保正确匹配
    LogStream& operator<<(const MatrixFormatter& formatter);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;  // 存储 logger 和 level（隐藏 spdlog 类型）
    std::ostringstream stream_;    // stream 在头文件中，支持模板方法
};

// ========================================================================
// LoggerHandle::log_fmt 辅助函数（模板函数必须在头文件中完整实现）
// ========================================================================
// 注意：由于 Impl 是不完整类型，我们不能直接调用其模板方法
// 解决方案：在头文件中使用 fmt::format 格式化字符串，然后调用非模板的 log 方法
// 这样既保持了 PIMPL，又实现了格式化功能
template <typename... Args>
inline auto logger_handle_log_fmt_impl(LoggerHandle* handle, LogLevel level, Args&&... args) -> void {
    if (handle) {
        // 使用 fmt::format 格式化字符串（fmt 库已在头文件中包含）
        std::string formatted = fmt::format(std::forward<Args>(args)...);
        // 调用非模板的 log 方法
        handle->log(level, formatted);
    }
}

// ========================================================================
// 统一日志接口 - 使用 if constexpr 实现
// ========================================================================
// 流式输出辅助函数
inline auto log_stream_impl(LogLevel level, const std::string& name) -> LogStream {
    return LogStream(LoggerManager::instance().get_logger(name), level);
}

// 格式化输出辅助函数（模板方法，需要在头文件中实现）
// 注意：spdlog 的 logger->log 已经支持格式化字符串，不需要手动调用 fmt::format
template <typename... Args>
inline auto log_fmt_impl(LogLevel level,
    const std::string& name,
    Args&&... args) -> void {
    auto logger = LoggerManager::instance().get_logger(name);
    if (logger) {
        // 直接调用 logger->log_fmt，spdlog 内部会处理格式化
        logger->log_fmt(level, std::forward<Args>(args)...);
    }
}

// 统一接口模板函数：使用 if constexpr 在编译期判断参数数量
template <typename... Args>
auto log_unified(LogLevel level, const std::string& name, Args&&... args) {
    constexpr size_t arg_count = sizeof...(Args);

    if constexpr (arg_count == 0) {
        // 编译期分支1：无额外参数 -> 流式输出
        // RTB_LOG_INFO("name") << "message"
        return log_stream_impl(level, name);
    } else {
        // 编译期分支2：有额外参数 -> 格式化输出
        // RTB_LOG_INFO("name", "message") 或 RTB_LOG_INFO("name", "format {}", value)
        // spdlog::fmt_lib::format 可以处理两种情况：
        // - 无占位符：format("message") -> "message"
        // - 有占位符：format("format {}", value) -> "format value"
        log_fmt_impl(level, name, std::forward<Args>(args)...);
    }
}

// ========================================================================
// 统一日志宏定义（宏作为前端，转发给模板函数）
// ========================================================================
// 使用方式：
// - 流式：RTB_LOG_INFO("name") << "message"
// - 格式化：RTB_LOG_INFO("name", "format: {}", value)
//           RTB_LOG_INFO("name", "simple message")  // 无占位符也可以
//
// 实现原理：
// 直接调用 log_unified 模板函数，它使用 if constexpr 在编译期判断参数数量
// 使用 ##__VA_ARGS__ 来处理空参数情况（GNU扩展，但广泛支持）
// 当 __VA_ARGS__ 为空时，##__VA_ARGS__ 会移除前面的逗号
// 当 __VA_ARGS__ 有参数时，##__VA_ARGS__ 正常展开
// 保存当前的警告设置
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvariadic-macros"
#endif

#define RTB_LOG_DEBUG(name, ...) \
    rtb::io::log_unified(rtb::io::LogLevel::Debug, name, ##__VA_ARGS__)
#define RTB_LOG_INFO(name, ...) \
    rtb::io::log_unified(rtb::io::LogLevel::Info, name, ##__VA_ARGS__)
#define RTB_LOG_WARN(name, ...) \
    rtb::io::log_unified(rtb::io::LogLevel::Warn, name, ##__VA_ARGS__)
#define RTB_LOG_ERROR(name, ...) \
    rtb::io::log_unified(rtb::io::LogLevel::Error, name, ##__VA_ARGS__)

// ============================================================================
// JSON 格式化辅助函数（用于 RTB_PLOT 和 RTB_JSON 宏）
// ============================================================================
namespace detail {

// =========================================================
// 对字符差串中的引号和反斜杠转义
// 用于确保 JSON 字符串可以安全地嵌入到 GELF 的 short_message 中
// =========================================================
/**
 * @brief 转义 JSON 字符串中的特殊字符
 * @details 将双引号、反斜杠等特殊字符转义，确保 JSON 字符串可以安全嵌入到 GELF 格式中
 */
inline auto escape_json_string(const std::string& input) -> std::string {
    std::string output;
    output.reserve(input.length() + 8);  // 预留一点空间，避免频繁重新分配

    for (char c : input) {
        switch (c) {
            case '"':
                output += "\\\"";
                break; /* " -> \" */
            case '\\':
                output += "\\\\";
                break; /* \ -> \\ */
            case '\b':
                output += "\\b";
                break; /* 退格 */
            case '\f':
                output += "\\f";
                break; /* 换页 */
            case '\n':
                output += "\\n";
                break; /* 换行 */
            case '\r':
                output += "\\r";
                break; /* 回车 */
            case '\t':
                output += "\\t";
                break; /* 制表符 */
            default:
                output += c;
                break;
        }
    }
    return output;
}

// =========================================================
// 智能 JSON 值格式化器
// =========================================================
/**
 * @brief 根据类型智能格式化 JSON 值
 * @details 算术类型（int, float, double, bool）不加引号，字符串类型自动加引号
 */
template <typename T>
inline auto format_json_value(const T& value) -> std::string {
    // 获取去除引用和 const 后的原始类型
    using U = std::decay_t<T>;

    if constexpr (std::is_arithmetic_v<U>) {
        // 情况A: 算术类型 (int, float, double, bool) -> 直接输出，不加引号
        // 注意：fmt 库会自动把 bool 格式化为 "true"/"false"，这也是合法的 JSON
        if constexpr (std::is_floating_point_v<U>) {
            // 浮点数：保留4位小数（与 RTB_PLOT 保持一致）
            return fmt::format("{:.4f}", value);
        } else {
            // 整数和布尔值：直接格式化
            return fmt::format("{}", value);
        }
    } else {
        // 情况B: 字符串或其他类型 (std::string, const char*, char*, string_view) -> 自动加引号
        // 这里的 \" 是转义双引号
        return fmt::format("\"{}\"", value);
    }
}

// =========================================================
// 键值对格式化器
// =========================================================
/**
 * @brief 格式化 JSON 键值对
 * @details 键始终加引号，值根据类型决定是否加引号
 */
template <typename T>
inline auto format_json_pair(const std::string& key, const T& value) -> std::string {
    // 逻辑： "Key": Value (Value 会根据类型决定是否带引号)
    return fmt::format("\"{}\": {}", key, format_json_value(value));
}

// 递归终止：处理最后一个键值对
template <typename Key, typename Value>
inline auto format_json_kv_pairs_impl(const Key& key, const Value& value) -> std::string {
    return format_json_pair(std::string(key), value);
}

// 递归展开：处理多个键值对
template <typename Key, typename Value, typename... Rest>
inline auto format_json_kv_pairs_impl(const Key& key, const Value& value, Rest&&... rest) -> std::string {
    return format_json_pair(std::string(key), value) + ", " + format_json_kv_pairs_impl(std::forward<Rest>(rest)...);
}

// 获取当前时间戳（秒，带小数部分）
inline auto get_timestamp() -> double {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() / 1e9;
    return seconds;
}
}  // namespace detail

// ============================================================================
// RTB_PLOT 宏：实时数据可视化接口
// ============================================================================
// 使用方式：
//   RTB_PLOT("plotter", "pos_x", 10.5, "vel_y", 0.35, "status", "Run");
// 自动生成 JSON：
//   {"t": 1234.5678, "pos_x": 10.5000, "vel_y": 0.3500, "status": "Run"}
//
// 注意：
//   1. 第一个参数是 logger 名称（必须已通过 add_udp_logger 创建）
//   2. 后续参数必须是成对的 (key, value)
//   3. 时间戳字段 "t" 会自动添加
//   4. 字符串值会自动添加引号，浮点数保留4位小数，整数和布尔值不加引号
#define RTB_PLOT(logger_name, ...)                                    \
    do {                                                              \
        double _t = rtb::io::detail::get_timestamp();                 \
        std::string _json = fmt::format("{{\"t\": {:.6f}, {}}}", _t,  \
            rtb::io::detail::format_json_kv_pairs_impl(__VA_ARGS__)); \
        RTB_LOG_INFO(logger_name, "{}", _json);                       \
    } while (0)

// ============================================================================
// 通用 JSON 日志宏 (Graylog 专用)
// ============================================================================
// 使用方式：
//   RTB_JSON("graylog", "event", "login", "uid", 1001, "latency", 25.5, "is_success", true);
//
// 效果：
//   生成的 JSON: {"event": "login", "uid": 1001, "latency": 25.5000, "is_success": true}
//   发送到 Graylog 时，JSON 字符串会被转义后放入 short_message 字段
//   (注意：不自动带时间戳t，因为graylog自带时间戳)
//   字符串值自动加引号，数字和布尔值不加引号，符合标准 JSON 格式
//
// 重要：生成的 JSON 字符串会被转义后嵌入到 GELF 格式的 short_message 中，
//       确保嵌套 JSON 格式正确（双引号会被转义为 \")
#define RTB_JSON(logger_name, ...)                                                  \
    do {                                                                            \
        std::string _json_raw = fmt::format("{{{}}}",                               \
            rtb::io::detail::format_json_kv_pairs_impl(__VA_ARGS__));               \
        std::string _json_escaped = rtb::io::detail::escape_json_string(_json_raw); \
        RTB_LOG_INFO(logger_name, "{}", _json_escaped);                             \
    } while (0)

// 恢复之前的警告设置
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#else  // RTB_LOG_ENABLED 未定义
// ============================================================================
// RTB_LOG_ENABLED 未定义 -> 使用 NullStream 提供流式输出兼容性
// ============================================================================
// 流式输出宏：返回 NullStream 对象，支持 << 操作符但不产生任何输出
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvariadic-macros"
#endif

#define RTB_LOG_DEBUG(name, ...) ((void)((void)0, ##__VA_ARGS__), rtb::io::NullStream())
#define RTB_LOG_INFO(name, ...) ((void)((void)0, ##__VA_ARGS__), rtb::io::NullStream())
#define RTB_LOG_WARN(name, ...) ((void)((void)0, ##__VA_ARGS__), rtb::io::NullStream())
#define RTB_LOG_ERROR(name, ...) ((void)((void)0, ##__VA_ARGS__), rtb::io::NullStream())
#define RTB_LOG_DEBUG_FMT(name, ...) ((void)0)
#define RTB_LOG_INFO_FMT(name, ...) ((void)0)
#define RTB_LOG_WARN_FMT(name, ...) ((void)0)
#define RTB_LOG_ERROR_FMT(name, ...) ((void)0)

#define RTB_PLOT(logger_name, ...) \
    do {                           \
    } while (0)

#define RTB_JSON(logger_name, ...) \
    do {                           \
    } while (0)

// 恢复之前的警告设置
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

// init_async 支持无参数调用（使用默认参数）
inline auto init_async(size_t queue_size = 8192, size_t thread_count = 1) -> void {
    (void)queue_size, (void)thread_count;
}

inline auto init_sync() -> void {}

inline auto add_console_logger(const std::string& name,
    const std::string& pattern = "[%l] %v") -> bool {
    (void)name, (void)pattern;
    return false;
}

inline bool add_file_logger(const std::string& name,
    const std::string& file_path,
    const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
    size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
    size_t max_files = 5) {                                // 保留文件数量
    (void)name, (void)file_path, (void)pattern, (void)max_file_size, (void)max_files;
    return false;
}

inline bool add_console_file_logger(const std::string& name,
    const std::string& file_path,
    const std::string& pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v",
    size_t max_file_size = SIZE_MAX - 1024 * 1024 * 1024,  // 避免溢出
    size_t max_files = 5) {                                // 保留文件数量
    (void)name, (void)file_path, (void)pattern, (void)max_file_size, (void)max_files;
    return false;
}

inline auto add_udp_logger(const std::string& name,
    const std::string& host = "127.0.0.1",
    uint16_t port = 9870) -> bool {
    (void)name, (void)host, (void)port;
    return false;
}

// LoggerHandle 的禁用版本（空类，提供兼容接口）
// 注意：必须在 LoggerManager 之前定义，因为 LoggerManager 需要使用它
class LoggerHandle {
public:
    auto flush() -> void {}

    auto set_level(int level) -> void { (void)level; }

    auto get_level() const -> int { return 0; }
};

inline auto add_graylog_logger(const std::string& name,
    const std::string& host = "127.0.0.1",
    uint16_t port = 12201,
    const std::string& source_host = "rtb") -> bool {
    (void)name, (void)host, (void)port, (void)source_host;
    return false;
}

inline auto update_logger_pattern(const std::string& name, const std::string& pattern) -> bool {
    (void)name, (void)pattern;
    return false;
}

inline auto get_logger(const std::string& name) -> std::shared_ptr<LoggerHandle> {
    (void)name;
    return nullptr;
}

inline auto remove_logger(const std::string& name) -> bool {
    (void)name;
    return false;
}

inline auto flush_all() -> void {}

inline auto shutdown() -> void {}

// LoggerManager 的禁用版本（空类，提供 instance() 方法以兼容代码）
class LoggerManager {
public:
    static auto instance() -> LoggerManager& {
        static LoggerManager instance;
        return instance;
    }

    auto flush_all() -> void {}

    auto shutdown() -> void {}

    auto get_logger(const std::string& name) -> std::shared_ptr<LoggerHandle> {
        (void)name;
        return nullptr;
    }

    auto update_logger_pattern(const std::string& name, const std::string& pattern) -> bool {
        (void)name, (void)pattern;
        return false;
    }

private:
    LoggerManager() = default;
    ~LoggerManager() = default;
    LoggerManager(const LoggerManager&) = delete;
    LoggerManager& operator=(const LoggerManager&) = delete;
    LoggerManager(LoggerManager&&) = delete;
    LoggerManager& operator=(LoggerManager&&) = delete;
};
#endif  // RTB_LOG_ENABLED

}  // namespace io
}  // namespace rtb

// ============================================================================
// 矩阵格式化支持
// ============================================================================
#ifdef RTB_LOG_ENABLED
// 包含 rtbio.hpp 以支持矩阵格式化（在命名空间外包含，避免循环依赖）
#include "io/rtbio.hpp"

// 实现 LogStream::operator<< 以支持 MatrixFormatter（在命名空间内）
namespace rtb {
namespace io {

// 实现 LogStream::operator<< 以支持 MatrixFormatter
// 注意：这是成员函数，可以访问 LogStream 的私有成员 stream_
inline LogStream& LogStream::operator<<(const MatrixFormatter& formatter) {
    // 成员函数可以直接访问私有成员 stream_
    // 使用临时流格式化矩阵，然后写入 stream_
    std::ostringstream temp_stream;
    print_matrix(formatter.data(), formatter.rows(), formatter.cols(),
        formatter.options(), temp_stream);
    // 通过 stream_ 直接写入字符串
    stream_ << temp_stream.str();
    return *this;
}

}  // namespace io
}  // namespace rtb

#endif  // RTB_LOG_ENABLED

#endif  // RTB_LOGGER_HPP