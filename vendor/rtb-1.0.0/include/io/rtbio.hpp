/**
 * @file rtbio.hpp
 * @brief RTB 输入输出工具库 (声明 + 模板实现)
 */

#ifndef RTB_IO_HPP
#define RTB_IO_HPP

#include <chrono>
#include <cstring>  // for memcpy
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace rtb {
namespace io {

// ============================================================================
// 文件路径辅助函数
// ============================================================================

/**
 * @brief 返回可执行文件路径向上回溯命中的首个可写 log 目录
 * @details 从可执行文件所在目录开始逐级向父目录回溯，查找已存在且可写的 "log" 目录。
 *          首次解析成功后会被静态缓存，后续调用不再重复搜索；若全链路未找到则报错并退出程序。
 * @return 带末尾路径分隔符的绝对或规范化目录路径
 */
auto getLogDirectory() -> std::string ;

/**
 * @brief 将文件名拼接到 getLogDirectory() 返回目录后，生成日志文件完整路径
 * @param filename 日志文件名（通常不含目录）
 * @return 可用于打开文件的完整路径字符串
 */
auto getLogFilePath(const std::string& filename) -> std::string ;

/**
 * @brief 获取带时间戳的文件名
 * @param prefix 文件名前缀
 * @param extension 扩展名（不带点）；省略或为空则不在末尾添加扩展名
 * @return prefix_时间戳[.extension]
 */
inline auto getTimeStampedFilePath(const std::string& prefix, const std::string& extension = "") -> std::string {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    std::string base = prefix + "_" + ss.str();
    if (extension.empty())
        return base;
    return base + "." + extension;
}

/**
 * @brief 在给定目录路径下，按扩展名筛选普通文件，并返回最近修改的一条的绝对路径
 * @param directory 要扫描的目录路径（存在且为目录时才会遍历其**直接子项**，不递归子目录）
 * @param extension 扩展名，可带点或不带点（例如 "csv"、".txt"）；与磁盘上后缀匹配时不区分大小写
 * @return 规范化后的绝对路径；directory 无效、无匹配文件、extension 为空、或遍历失败时返回空字符串
 * @note “最新”按文件系统记录的 last_write_time（最后写入时间）比较；无跨平台统一的文件创建时间 API。
 */
auto findNewestFileInDirectory(const std::string& directory, const std::string& extension) -> std::string ;

// ============================================================================
// 1. 打印风格定义
// ============================================================================

enum class PrintStyle {
    Plain,
    Matlab,
    Python,
    Numpy,
    Cpp,
    Json,
};

struct PrintOptions {
    int precision = 15;
    int indent = 4;
    int cell_width = 20;
    PrintStyle style = PrintStyle::Cpp;
    std::string name = "";
    bool fixed = true;
};

// 内部 Traits
struct FormatTraits {
    std::string matrix_start, matrix_end;
    std::string row_start, row_end;
    std::string col_sep, row_sep;
    std::string var_suffix, stmt_end;
};

FormatTraits get_traits(PrintStyle style);

// ============================================================================
// 2. 核心打印函数
// ============================================================================

/**
 * @brief 通用矩阵打印函数
 */
auto print_matrix(
    const double* data, int rows, int cols, PrintOptions opts = PrintOptions(), std::ostream& os = std::cout) -> void ;

// 具体类型的便捷声明
auto print_mat4(const double* m, const std::string& name = "T", PrintStyle style = PrintStyle::Matlab) -> void ;
auto print_vec(const double* v, int n, const std::string& name = "v", PrintStyle style = PrintStyle::Matlab) -> void ;

// ============================================================================
// 3. 数据类型转换工具
// ============================================================================

// 将 2D vector 展平为 1D vector
auto flatten(const std::vector<std::vector<double>>& matrix) -> std::vector<double> ;

// 将 2D vector 复制到 raw double* buffer 中 (需确保 buffer 够大)
// 返回复制的元素个数
auto copy_to_buffer(const std::vector<std::vector<double>>& matrix, double* buffer, size_t buffer_size) -> size_t ;

// 将 raw double* 转换为 2D vector
auto unflatten(const double* data, int rows, int cols) -> std::vector<std::vector<double>> ;

// ============================================================================
// 4. DataLogger: 实时数据记录器
// ============================================================================

class DataLogger {
public:
    DataLogger(const std::string& filename, const std::string& header = "");
    ~DataLogger();

    auto log(const std::vector<double>& row) -> void ;
    auto log(const double* data, size_t size) -> void ;

    // 模板函数必须在头文件
    template <size_t N> void log(const double (&data)[N]) {
        // 如果未打开，直接返回，避免崩溃
        log(data, N);
    }

    auto flush() -> void ;

private:
    auto write_line_internal(const double* data, size_t size) -> void ;
    std::string filename_;
    std::ofstream file_;
};

// ============================================================================
// 5. 文件读写函数 (声明)
// ============================================================================

auto save_to_csv(const std::string& filename, const std::vector<std::vector<double>>& data, bool append = false,
    const std::string& header = "") -> bool ;

auto load_csv(const std::string& filename, std::vector<std::vector<double>>& out_data, bool skip_header = false) -> bool ;

auto load_matrix_fixed(const std::string& filename, double* buffer, int expected_size, bool skip_header = false) -> bool ;

auto load_mat4(const std::string& filename, double* T_out) -> bool ;

// ============================================================================
// 6. 矩阵格式化包装类（用于日志输出）
// ============================================================================

/**
 * @brief 矩阵格式化包装类，用于将矩阵/数组输出到日志
 * @details 包装矩阵数据和打印选项，支持通过 << 操作符输出到 LogStream
 */
class MatrixFormatter {
public:
    MatrixFormatter(const double* data, int rows, int cols, const PrintOptions& opts = PrintOptions())
        : data_(data), rows_(rows), cols_(cols), opts_(opts) {}

    // 便捷构造函数：使用默认选项，只指定风格
    MatrixFormatter(const double* data, int rows, int cols, PrintStyle style) : data_(data), rows_(rows), cols_(cols) {
        opts_.style = style;
    }

    // 访问器
    auto data() const -> const double* { return data_; }

    auto rows() const -> int { return rows_; }

    auto cols() const -> int { return cols_; }

    auto options() const -> const PrintOptions& { return opts_; }

private:
    const double* data_;
    int rows_;
    int cols_;
    PrintOptions opts_;
};

/**
 * @brief 便捷函数：创建矩阵格式化器
 * @param data 矩阵数据指针
 * @param rows 行数
 * @param cols 列数
 * @param opts 打印选项
 * @return MatrixFormatter 对象
 */
inline auto format_matrix(
    const double* data, int rows, int cols, const PrintOptions& opts = PrintOptions()) -> MatrixFormatter {
    return MatrixFormatter(data, rows, cols, opts);
}

/**
 * @brief 便捷函数：创建矩阵格式化器（指定风格）
 * @param data 矩阵数据指针
 * @param rows 行数
 * @param cols 列数
 * @param style 打印风格
 * @return MatrixFormatter 对象
 */
inline auto format_matrix(const double* data, int rows, int cols, PrintStyle style) -> MatrixFormatter {
    return MatrixFormatter(data, rows, cols, style);
}

// ============================================================================
// 7. 宏定义
// ============================================================================
#define RTB_PRINT(var, r, c)                                                                                           \
    {                                                                                                                  \
        rtb::io::PrintOptions _o;                                                                                      \
        _o.name = #var;                                                                                                \
        _o.style = rtb::io::PrintStyle::Plain;                                                                         \
        rtb::io::print_matrix(var, r, c, _o);                                                                          \
    }

#define RTB_PRINT_MATLAB(var, r, c)                                                                                    \
    {                                                                                                                  \
        rtb::io::PrintOptions _o;                                                                                      \
        _o.name = #var;                                                                                                \
        _o.style = rtb::io::PrintStyle::Matlab;                                                                        \
        rtb::io::print_matrix(var, r, c, _o);                                                                          \
    }

#define RTB_PRINT_CPP(var, r, c)                                                                                       \
    {                                                                                                                  \
        rtb::io::PrintOptions _o;                                                                                      \
        _o.name = #var;                                                                                                \
        _o.style = rtb::io::PrintStyle::Cpp;                                                                           \
        rtb::io::print_matrix(var, r, c, _o);                                                                          \
    }

#define RTB_PRINT_PY(var, r, c)                                                                                        \
    {                                                                                                                  \
        rtb::io::PrintOptions _o;                                                                                      \
        _o.name = #var;                                                                                                \
        _o.style = rtb::io::PrintStyle::Python;                                                                        \
        rtb::io::print_matrix(var, r, c, _o);                                                                          \
    }

#define RTB_LOG_M4(var) rtb::io::print_mat4(var, #var, rtb::io::PrintStyle::Matlab)

/**
 * @brief 抛出文件行号错误 并打印错误信息
 * @param error 错误信息
 */
#ifndef THROW_FILE_LINE
#define THROW_FILE_LINE(error)                                                                                         \
    throw std::runtime_error(std::string(__FILE__) + " : " + std::to_string(__LINE__) + " : " + error)
#endif

}  // namespace io
}  // namespace rtb

#endif  // RTB_IO_HPP