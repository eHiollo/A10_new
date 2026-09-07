/**
 * @file rtb_math.hpp
 * @brief RTB统一数学工具库 - 向量、矩阵、四元数、最小二乘等数学函数
 * @details 支持内联优化、模板函数、函数重载
 *  符号定义(暂未完全实施):
 *    pp  :  3x1 点位置(position of point)
 *    re  :  3x1 欧拉角(eula angle)
 *    rq  :  4x1 四元数(quaternions) [qw, qx, qy, qz]
 *    rm  :  3x3 旋转矩阵(rotation matrix)
 *    pe  :  6x1 点位置与欧拉角(position and eula angle)
 *    pq  :  7x1 点位置与四元数(position and quaternions)
 *    pm  :  4x4 位姿矩阵(pose matrix)
 *    ra  :  3x1 绕固定轴的旋转的指数积（rotation around axis, exponential product）
 *    ps  :  6x1 位移螺旋，单位速度螺旋[vu;wu]转theta角，也就是ps =[v,w]=[vu;wu]*theta, theta = |w|
 *
 *    vp  :  3x1 线速度(velocity of point)
 *    we  :  3x1 欧拉角导数(omega in term of eula angle)
 *    wq  :  4x1 四元数导数(omega in term of quternions)
 *    wm  :  3x3 旋转矩阵导数(omega in term of rotation matrix)
 *    ve  :  6x1 线速度与欧拉角导数（velocity and omega in term of eula angle）
 *    vq  :  7x1 线速度与四元数导数(velocity and omega in term of quternions)
 *    vm  :  4x4 位姿矩阵导数(velocity in term of pose matrix)
 *    wa  :  3x1 角速度(omega)
 *    va  :  6x1 线速度与角速度(velocity and omega)
 *    vs  :  6x1 螺旋速度(velocity screw)
 *
 *    ap  :  3x1 线加速度(acceleration of point)
 *    xe  :  3x1 欧拉角导导数(alpha in term of eula angle)
 *    xq  :  4x1 四元数导导数(alpha in term of quternions)
 *    xm  :  3x3 旋转矩阵导导数(alpha in term of rotation matrix)
 *    ae  :  6x1 线加速度与欧拉角导导数(acceleration and alpha in term of eula angle)
 *    aq  :  7x1 线加速度与四元数导导数(acceleration and alpha in term of quternions)
 *    am  :  4x4 位姿矩阵导导数(acceleration in term of pose matrix)
 *    xa  :  3x1 角加速度(alpha, acceleration of angle)
 *    aa  :  6x1 线加速度与角加速度(acceleration and alpha)
 *    as  :  6x1 螺旋加速度(acceleration screw)
 *
 *    i3  :  3x3 惯量矩阵
 *    im  :  6x6 空间惯量矩阵
 *    iv  :  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
 */

#ifndef RTB_MATH_HPP
#define RTB_MATH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#if defined(__GNUC__) || defined(__clang__)
#define RTB_RESTRICT __restrict__
#elif defined(_MSC_VER)
#define RTB_RESTRICT __restrict
#else
#define RTB_RESTRICT
#endif

namespace rtb {
namespace math {

// ============================================================================
// 基础数据类型（数组长度约定：Vec3=3, Mat3x3=9行优先, Mat4x4=16行优先, PE=6, Joint=7）
// ============================================================================
using Interval = std::array<double, 2>;  // [min, max]

// ============================================================================
// 常量定义
// ============================================================================
constexpr double PI = 3.14159265358979323846;
constexpr double PI_2 = 2.0 * PI;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;
constexpr double EPSILON = 1e-14;

// ============================================================================
// 常用函数
// ============================================================================

/**
 * @brief 符号函数
 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief 输出饱和函数
 */
template <typename T> T output_limit(T input, T limit, T offset) {
    return std::max(-limit + offset, std::min(limit + offset, input));
}

/**
 * @brief 安全的sqrt函数，避免负数
 */
constexpr inline auto safe_sqrt(double x) noexcept -> double {
    return (x < 0.0) ? 0.0 : std::sqrt(x);
}

// ============================================================================
// 三角函数
// ============================================================================

/**
 * @brief 安全的asin函数，避免输入超出[-1,1]范围
 */
constexpr inline auto safe_asin(double x) noexcept -> double {
    if (x > 1.0)
        return PI / 2.0;
    if (x < -1.0)
        return -PI / 2.0;
    return std::asin(x);
}

/**
 * @brief 安全的acos函数，避免输入超出[-1,1]范围
 */
constexpr inline auto safe_acos(double x) noexcept -> double {
    if (x > 1.0)
        return 0.0;
    if (x < -1.0)
        return PI;
    return std::acos(x);
}

inline auto fast_sincos(double theta, double& s, double& c) -> void {
#if defined(__linux__)
    // 只在 Linux 环境下 (GCC/Clang with glibc) 使用内置加速
    __builtin_sincos(theta, &s, &c);
#elif defined(__APPLE__)
    // macOS 环境 (Apple Clang)
    // 苹果系统自带的 math 库没有 sincos，且 __builtin_sincos 不可用。
    // 但不用担心，Apple Clang 的优化器非常强，在 -O2/-O3 下
    // 它会自动把下面两行合并成一条高效指令 (如 ARM64 的指令优化)。
    s = std::sin(theta);
    c = std::cos(theta);
#else
    // Windows (MSVC) 或其他平台
    s = std::sin(theta);
    c = std::cos(theta);
#endif
}

/**
 * @brief 角度归一化到 (-π, π]
 */
inline auto toPI(double angle) -> double {
    // [-π, π]
    double s, c;
    fast_sincos(angle, s, c);
    angle = std::atan2(s, c);

    // (-π, π]
    if (angle <= -PI + EPSILON)
        angle += 2.0 * PI;
    else if (angle > PI - EPSILON)
        angle -= 2.0 * PI;
    return angle;
}

/**
 * @brief 鲁棒的 atan2 计算，通过引入阻尼项融合参考角度，消除奇异点附近的跳变。
 * * 原理：计算 V_measure + V_damping 的合成向量的角度。
 * V_measure = (x, y)          -> 来自当前运动学计算（在奇异点趋近 0）
 * V_damping = lambda * UnitVec(ref) -> 来自上一时刻的参考方向
 * * @param y       原始 atan2 的 y 分量 (通常包含 sin(q2) 因子)
 * @param x       原始 atan2 的 x 分量 (通常包含 sin(q2) 因子)
 * @param ref     参考角度 (上一时刻的关节角 q_prev)
 * @param lambda  阻尼因子 (建议 1e-6 ~ 1e-4)
 * @return double 计算出的平滑角度 (-pi, pi]
 */
inline auto atan2_robust(double y, double x, double ref, double lambda = 1e-10) -> double {
    // 1. 根据参考角度构建“引导向量” (Damping Vector)
    // 注意：这里假设 lambda 相对于 x, y 的最大值（通常是1.0）是很小的
    double s, c;
    fast_sincos(ref, s, c);
    double x_damp = lambda * c;
    double y_damp = lambda * s;

    // 2. 向量融合 (Vector Fusion)
    // 当 (x,y) 很大时，damp 项几乎不影响结果（保留精度）
    // 当 (x,y) 趋近 0 时，damp 项主导结果（平滑过渡到 ref）
    double x_new = x + x_damp;
    double y_new = y + y_damp;

    // 3. 计算合成向量的角度
    return std::atan2(y_new, x_new);
}

//
// 返回解的个数，解存入 roots
/**
 * @brief 求解方程 A*sin(psi) + B*cos(psi) + C = 0
 * @param A 系数
 * @param B 系数
 * @param C 系数
 * @return roots[2]方程的解
 */
inline auto solveTrigEq(double A, double B, double C, double* roots) -> int {
    double dist_sq = A * A + B * B;
    if (dist_sq < 1e-9)
        return 0;  // 避免除零

    // 归一化: sin(psi + alpha) = -C / sqrt(A^2+B^2)
    double r = std::sqrt(dist_sq);
    double alpha = std::atan2(B, A);
    double rhs = -C / r;

    if (std::abs(rhs) > 1.0)
        return 0;  // 无解

    double t = std::acos(rhs);  // 0 到 PI

    // psi + alpha = t  OR  psi + alpha = -t
    roots[0] = toPI(t - alpha);
    roots[1] = toPI(-t - alpha);
    return 2;
}

// ============================================================================
// 通用向量运算函数
// ============================================================================

/**
 * @brief 向量清零
 */
template <typename T> inline void vec_zero(T* v, int n) {
    std::memset(v, 0, n * sizeof(T));
}

/**
 * @brief 向量拷贝 dst = src
 */
template <typename T> inline void vec_copy(const T* src, T* dst, int n) {
    std::memcpy(dst, src, n * sizeof(T));
}

/**
 * @brief 带跨度的拷贝 (Stride Copy)
 */
template <typename T> constexpr inline void vec_copy(int n, const T* src, T* dst, int stride_dst) {
    for (int i = 0; i < n; ++i) {
        dst[i * stride_dst] = src[i];
    }
}

/**
 * @brief 向量数乘 dst = s * src
 */
template <typename T> constexpr inline void vec_scale(const T* src, T s, T* dst, int n) {
    for (int i = 0; i < n; ++i)
        dst[i] = src[i] * s;
}

/**
 * @brief 向量数乘（原地） v *= s
 */
template <typename T> inline void vec_scale(T* v, T s, int n) {
    for (int i = 0; i < n; ++i)
        v[i] *= s;
}

/**
 * @brief 向量加法 dst = a + b
 */
template <typename T> constexpr inline void vec_add(const T* a, const T* b, T* dst, int n) {
    for (int i = 0; i < n; ++i)
        dst[i] = a[i] + b[i];
}

/**
 * @brief 向量加法（累加） dst += src
 */
template <typename T> constexpr inline void vec_add(const T* src, T* dst, int n) {
    for (int i = 0; i < n; ++i)
        dst[i] += src[i];
}

/**
 * @brief 向量减法 dst = a - b
 */
template <typename T> constexpr inline void vec_sub(const T* a, const T* b, T* dst, int n) {
    for (int i = 0; i < n; ++i)
        dst[i] = a[i] - b[i];
}

/**
 * @brief 向量模长
 */
template <typename T> constexpr inline T vec_norm(const T* v, int n) {
    T sum = 0;
    for (int i = 0; i < n; ++i)
        sum += v[i] * v[i];
    return safe_sqrt(sum);
}

/**
 * @brief 向量点积
 */
template <typename T> constexpr inline T vec_dot(const T* a, const T* b, int n) {
    T sum = 0;
    for (int i = 0; i < n; ++i)
        sum += a[i] * b[i];
    return sum;
}

/**
 * @brief 向量归一化（原地）
 */
template <typename T> constexpr inline void vec_normalize(T* v, int n, T eps = EPSILON) {
    T nm = vec_norm(v, n);
    if (nm > eps) {
        for (int i = 0; i < n; ++i)
            v[i] /= nm;
    }
}

/**
 * @brief 向量归一化并缩放到指定模长
 */
template <typename T> inline void vec_normalize_scale(T* v, int n, T scale, T eps = EPSILON) {
    T nm = vec_norm(v, n);
    if (nm > eps) {
        T factor = scale / nm;
        for (int i = 0; i < n; ++i)
            v[i] *= factor;
    } else {
        vec_zero(v, n);
    }
}

// 限制模长的辅助函数
template <typename T> inline void limit_norm(T* vec, int n, T max_val) {
    double norm = vec_norm(vec, n);
    // 手动计算 norm 以防万一
    if (n == 3)
        norm = safe_sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

    if (norm > max_val && norm > EPSILON) {
        double scale = max_val / norm;
        for (int i = 0; i < n; ++i)
            vec[i] *= scale;
    }
}

// ============================================================================
// 3D向量数学运算函数
// ============================================================================

/**
 * @brief 3D向量加法 out = a + b
 */
constexpr inline auto vec3_add(const double* a, const double* b, double* out) -> void {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

/**
 * @brief 3D向量减法 out = a - b
 */
constexpr inline auto vec3_sub(const double* a, const double* b, double* out) -> void {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

/**
 * @brief 3D向量数乘 out = s * v
 */
constexpr inline auto vec3_scale(const double* v, double s, double* out) -> void {
    out[0] = v[0] * s;
    out[1] = v[1] * s;
    out[2] = v[2] * s;
}

/**
 * @brief 3D向量模长
 */
constexpr inline auto vec3_norm(const double* v) -> double {
    return safe_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * @brief 3D向量归一化（原地）
 */
inline auto vec3_normalize(double* v, double eps = EPSILON) -> void {
    double n = vec3_norm(v);
    if (n > eps) {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
    }
}

/**
 * @brief 3D向量点积
 */
constexpr inline auto vec3_dot(const double* a, const double* b) -> double {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * @brief 3D向量叉乘 result = a × b
 */
constexpr inline auto vec3_cross(const double* a, const double* b, double* result) -> void {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// ============================================================================
// 通用矩阵运算函数
// A, B, C 均为行优先存储的一维数组指针
// ============================================================================

/**
 * @brief 矩阵清零
 */
inline auto mat_zero(double* A, int n) -> void {
    std::memset(A, 0, n * sizeof(double));
}

/**
 * @brief 单位矩阵
 */
inline auto mat_eye(double* A, int n) -> void {
    std::fill(A, A + n * n, 0.0);
    for (int i = 0; i < n; ++i)
        A[i * n + i] = 1.0;
}

/**
 * @brief 矩阵加法 C = A + B
 */
constexpr inline auto mat_add(
    const double* RTB_RESTRICT A, const double* RTB_RESTRICT B, double* RTB_RESTRICT C, int rows, int cols) -> void {
    for (int i = 0; i < rows * cols; ++i)
        C[i] = A[i] + B[i];
}

/**
 * @brief 矩阵减法 C = A - B
 */
constexpr inline auto mat_sub(
    const double* RTB_RESTRICT A, const double* RTB_RESTRICT B, double* RTB_RESTRICT C, int rows, int cols) -> void {
    for (int i = 0; i < rows * cols; ++i)
        C[i] = A[i] - B[i];
}

/**
 * @brief 矩阵数乘 C = k * A
 */
constexpr inline auto mat_scale(const double* A, double* B, int rows, int cols, double k) -> void {
    for (int i = 0; i < rows * cols; ++i)
        B[i] = A[i] * k;
}

/**
 * @brief 矩阵乘法 C = A * B
 */
constexpr inline auto mat_mul(const double* RTB_RESTRICT A, const double* RTB_RESTRICT B, double* RTB_RESTRICT C,
    int rowsA, int colsA, int colsB) -> void {
    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsB; ++j) {
            double sum = 0.0;
            for (int k = 0; k < colsA; ++k) {
                sum += A[i * colsA + k] * B[k * colsB + j];
            }
            C[i * colsB + j] = sum;
        }
    }
}

/**
 * @brief 矩阵转置 B = A^T (A: m×n, B: n×m)
 */
constexpr inline auto mat_transpose(const double* A, double* B, int m, int n) -> void {
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            B[j * m + i] = A[i * n + j];
        }
    }
}

/**
 * @brief 矩阵向量乘法 y = A * x (A: m×n, x: n×1)
 */
constexpr inline auto mat_vec_mul(const double* A, const double* x, double* y, int m, int n) -> void {
    for (int i = 0; i < m; ++i) {
        double sum = 0.0;
        for (int j = 0; j < n; ++j) {
            sum += A[i * n + j] * x[j];
        }
        y[i] = sum;
    }
}

/**
 * @brief 矩阵求逆 C = A^-1
 */
auto mat_inv(const double* RTB_RESTRICT A, double* RTB_RESTRICT B, int n) -> bool ;

// ============================================================================
// 3x3 矩阵运算函数
// ============================================================================

/**
 * @brief 创建单位3×3旋转矩阵（单位矩阵）
 */
inline auto mat3_identity(double* R) -> void {
    std::memset(R, 0, 9 * sizeof(double));
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
}

/**
 * @brief 3×3矩阵转置特化
 */
constexpr inline auto mat3_transpose(const double* RTB_RESTRICT A, double* RTB_RESTRICT At) -> void {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            At[i * 3 + j] = A[j * 3 + i];
}

/**
 * @brief 3×3矩阵向量乘法特化
 */
constexpr inline auto mat3_vec_mul(const double* A, const double* x, double* y) -> void {
    for (int i = 0; i < 3; ++i) {
        y[i] = A[i * 3 + 0] * x[0] + A[i * 3 + 1] * x[1] + A[i * 3 + 2] * x[2];
    }
}

/**
 * @brief 3×3矩阵乘法 C = A * B
 */
constexpr inline auto mat3_mul(const double* RTB_RESTRICT A, const double* RTB_RESTRICT B, double* RTB_RESTRICT C) -> void {
    C[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
    C[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
    C[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
    C[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
    C[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
    C[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
    C[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
    C[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
    C[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

/**
 * @brief 计算 3x3 矩阵的行列式
 *
 */
constexpr inline auto mat3_det(const double* A) -> double {
    return A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) + A[2] * (A[3] * A[7] - A[4] * A[6]);
}

/**
 * @brief 计算 3x3 矩阵的迹 (Trace)
 */
constexpr inline auto mat3_trace(const double* R) -> double {
    return R[0] + R[4] + R[8];
}

/**
 * @brief 快速3x3矩阵求逆 (基于克拉默法则，栈上计算，无动态内存)
 * @param A 输入矩阵 [9] (行优先)
 * @param A_inv 输出逆矩阵 [9] (行优先)
 * @return true 成功, false 奇异矩阵
 */
inline auto mat3_inv_fast(const double* RTB_RESTRICT A, double* RTB_RESTRICT A_inv) -> bool {
    double det = mat3_det(A);
    if (std::abs(det) < EPSILON)
        return false;

    double inv_det = 1.0 / det;

    // 克拉默法则：A^-1 = (1/det) * C^T，其中C是余子式矩阵
    A_inv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    A_inv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    A_inv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    A_inv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    A_inv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    A_inv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    A_inv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    A_inv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    A_inv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;

    return true;
}

/**
 * @brief Gram-Schmidt正交化 (快速修正旋转矩阵)
 * @param R_in 输入矩阵 [9] (行优先)
 * @param R_out 输出正交化矩阵 [9] (行优先)
 * @details 逻辑：
 *   1. 第0列（X轴）归一化
 *   2. 第1列（Y轴）减去在X轴投影，归一化
 *   3. 第2列（Z轴）由 X cross Y 生成（确保右手系）
 */
inline auto mat3_orth_gram_schmidt(const double* RTB_RESTRICT R_in, double* RTB_RESTRICT R_out) -> void {
    // 提取列向量（注意：输入是行优先，列向量索引跨度为3）
    double x[3] = {R_in[0], R_in[3], R_in[6]};
    double y[3] = {R_in[1], R_in[4], R_in[7]};
    double z[3] = {R_in[2], R_in[5], R_in[8]};

    // 1. 归一化X轴
    double x_norm = vec3_norm(x);
    if (x_norm < EPSILON) {
        mat3_identity(R_out);
        return;
    }
    x[0] /= x_norm;
    x[1] /= x_norm;
    x[2] /= x_norm;

    // 2. Y轴减去在X轴上的投影，然后归一化
    double proj = vec3_dot(y, x);
    y[0] -= proj * x[0];
    y[1] -= proj * x[1];
    y[2] -= proj * x[2];
    double y_norm = vec3_norm(y);
    if (y_norm < EPSILON) {
        // Y轴退化，重新生成
        if (std::abs(x[2]) < 0.9) {
            y[0] = 0;
            y[1] = 0;
            y[2] = 1;
        } else {
            y[0] = 1;
            y[1] = 0;
            y[2] = 0;
        }
        proj = vec3_dot(y, x);
        y[0] -= proj * x[0];
        y[1] -= proj * x[1];
        y[2] -= proj * x[2];
        y_norm = vec3_norm(y);
    }
    y[0] /= y_norm;
    y[1] /= y_norm;
    y[2] /= y_norm;

    // 3. Z轴由 X cross Y 生成（确保右手系）
    vec3_cross(x, y, z);
    double z_norm = vec3_norm(z);
    if (z_norm < EPSILON) {
        mat3_identity(R_out);
        return;
    }
    z[0] /= z_norm;
    z[1] /= z_norm;
    z[2] /= z_norm;

    // 组装输出矩阵（行优先）
    R_out[0] = x[0];
    R_out[1] = y[0];
    R_out[2] = z[0];
    R_out[3] = x[1];
    R_out[4] = y[1];
    R_out[5] = z[1];
    R_out[6] = x[2];
    R_out[7] = y[2];
    R_out[8] = z[2];
}

/**
 * @brief 牛顿迭代极分解 (Polar Decomposition)
 * @param A 输入任意非奇异3x3矩阵 [9] (行优先)
 * @param R 输出正交旋转矩阵 [9] (行优先)
 * @param max_iter 最大迭代次数 (默认10)
 * @details 公式：R_{k+1} = 0.5 * (R_k + R_k^{-T})
 *          使用栈数组，无动态内存分配
 */
inline auto polar_decomposition_newton(const double* RTB_RESTRICT A, double* RTB_RESTRICT R, int max_iter = 10) -> void {
    // 初始化：R = A
    double R_work[9];
    std::memcpy(R_work, A, 9 * sizeof(double));

    double Ri_inv[9];
    double Ri_inv_T[9];

    for (int iter = 0; iter < max_iter; ++iter) {
        // 计算 R^{-1}
        if (!mat3_inv_fast(R_work, Ri_inv))
            break;  // 奇异，退出

        // 计算 (R^{-1})^T = R^{-T}
        mat3_transpose(Ri_inv, Ri_inv_T);

        // R_new = 0.5 * (R + R^{-T})
        for (int i = 0; i < 9; ++i) {
            R_work[i] = 0.5 * (R_work[i] + Ri_inv_T[i]);
        }
    }

    // 将结果拷贝到输出
    std::memcpy(R, R_work, 9 * sizeof(double));
}

/**
 * @brief 修正旋转矩阵漂移 (恢复正交归一性)
 * @param R 输入输出矩阵 [9] (行优先)
 * @details 内部调用 polar_decomposition_newton
 */
inline auto mat3_fix_rotation(double* R) -> void {
    double R_fixed[9];
    polar_decomposition_newton(R, R_fixed, 5);
    // 将修复后的数据拷回原数组
    std::memcpy(R, R_fixed, 9 * sizeof(double));
}

// ============================================================================
// 4x4 矩阵运算函数
// ============================================================================

/**
 * @brief 创建单位4×4齐次变换矩阵
 * @param T 输出的4×4单位变换矩阵
 */
inline auto mat4_identity(double* T) -> void {
    std::memset(T, 0, 16 * sizeof(double));
    T[0] = 1.0;
    T[5] = 1.0;
    T[10] = 1.0;
    T[15] = 1.0;
}

/**
 * @brief 4×4齐次矩阵乘法 C = A * B
 */
constexpr inline auto mat4_mul(const double* RTB_RESTRICT A, const double* RTB_RESTRICT B, double* RTB_RESTRICT C) -> void {
    C[0] = A[0] * B[0] + A[1] * B[4] + A[2] * B[8] + A[3] * B[12];
    C[1] = A[0] * B[1] + A[1] * B[5] + A[2] * B[9] + A[3] * B[13];
    C[2] = A[0] * B[2] + A[1] * B[6] + A[2] * B[10] + A[3] * B[14];
    C[3] = A[0] * B[3] + A[1] * B[7] + A[2] * B[11] + A[3] * B[15];
    C[4] = A[4] * B[0] + A[5] * B[4] + A[6] * B[8] + A[7] * B[12];
    C[5] = A[4] * B[1] + A[5] * B[5] + A[6] * B[9] + A[7] * B[13];
    C[6] = A[4] * B[2] + A[5] * B[6] + A[6] * B[10] + A[7] * B[14];
    C[7] = A[4] * B[3] + A[5] * B[7] + A[6] * B[11] + A[7] * B[15];
    C[8] = A[8] * B[0] + A[9] * B[4] + A[10] * B[8] + A[11] * B[12];
    C[9] = A[8] * B[1] + A[9] * B[5] + A[10] * B[9] + A[11] * B[13];
    C[10] = A[8] * B[2] + A[9] * B[6] + A[10] * B[10] + A[11] * B[14];
    C[11] = A[8] * B[3] + A[9] * B[7] + A[10] * B[11] + A[11] * B[15];
    C[12] = A[12] * B[0] + A[13] * B[4] + A[14] * B[8] + A[15] * B[12];
    C[13] = A[12] * B[1] + A[13] * B[5] + A[14] * B[9] + A[15] * B[13];
    C[14] = A[12] * B[2] + A[13] * B[6] + A[14] * B[10] + A[15] * B[14];
    C[15] = A[12] * B[3] + A[13] * B[7] + A[14] * B[11] + A[15] * B[15];
}

/**
 * @brief 从4×4齐次矩阵提取3×3旋转矩阵 (行优先)
 */
constexpr inline auto pm2rm(const double* RTB_RESTRICT pm, double* RTB_RESTRICT R) -> void {
    R[0] = pm[0];
    R[1] = pm[1];
    R[2] = pm[2];
    R[3] = pm[4];
    R[4] = pm[5];
    R[5] = pm[6];
    R[6] = pm[8];
    R[7] = pm[9];
    R[8] = pm[10];
}

/**
 * @brief 4×4齐次矩阵求逆
 * @param T 输入的4×4齐次变换矩阵
 * @param T_inv 输出的逆矩阵
 * @return true=成功, false=矩阵奇异
 *
 * @details 齐次变换矩阵形式：
 *   T = [R  p]    T^-1 = [R^T  -R^T*p]
 *       [0  1]            [0    1     ]
 */
constexpr inline auto mat4_inv(const double* RTB_RESTRICT T, double* RTB_RESTRICT T_inv) -> bool {
    // 提取旋转矩阵 R (3x3)
    double R[9]{};
    pm2rm(T, R);

    // 提取位置向量 p
    double p[3] = {T[3], T[7], T[11]};

    // 计算 R^T
    double Rt[9]{};
    mat3_transpose(R, Rt);

    // 计算 -R^T * p
    double Rt_p[3]{};
    mat3_vec_mul(Rt, p, Rt_p);
    Rt_p[0] = -Rt_p[0];
    Rt_p[1] = -Rt_p[1];
    Rt_p[2] = -Rt_p[2];

    // 组装逆矩阵
    T_inv[0] = Rt[0];
    T_inv[1] = Rt[1];
    T_inv[2] = Rt[2];
    T_inv[3] = Rt_p[0];
    T_inv[4] = Rt[3];
    T_inv[5] = Rt[4];
    T_inv[6] = Rt[5];
    T_inv[7] = Rt_p[1];
    T_inv[8] = Rt[6];
    T_inv[9] = Rt[7];
    T_inv[10] = Rt[8];
    T_inv[11] = Rt_p[2];
    T_inv[12] = 0.0;
    T_inv[13] = 0.0;
    T_inv[14] = 0.0;
    T_inv[15] = 1.0;

    return true;  // 齐次变换矩阵总是可逆的（除非R不是旋转矩阵）
}

// ============================================================================
// 6x6 矩阵运算函数
// ============================================================================

/**
 * @brief 6×6矩阵求逆 (高斯-约旦消元法)
 * @return true=成功, false=奇异
 */
inline auto mat6_inv(const double* RTB_RESTRICT A, double* RTB_RESTRICT A_inv) -> bool {
    int n = 6;
    double M[36 * 2];  // 增广矩阵 [A | I]

    // 初始化增广矩阵
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            M[i * (2 * n) + j] = A[i * n + j];
            M[i * (2 * n) + (j + n)] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 高斯消元
    for (int k = 0; k < n; ++k) {
        // 寻找主元
        int pivot = k;
        double max_val = std::abs(M[k * (2 * n) + k]);
        for (int i = k + 1; i < n; ++i) {
            if (std::abs(M[i * (2 * n) + k]) > max_val) {
                max_val = std::abs(M[i * (2 * n) + k]);
                pivot = i;
            }
        }

        if (max_val < EPSILON)
            return false;  // 奇异矩阵

        // 交换行
        if (pivot != k) {
            for (int j = 0; j < 2 * n; ++j) {
                std::swap(M[k * (2 * n) + j], M[pivot * (2 * n) + j]);
            }
        }

        // 归一化当前行
        double diag = M[k * (2 * n) + k];
        for (int j = k; j < 2 * n; ++j) {
            M[k * (2 * n) + j] /= diag;
        }

        // 消去其他行
        for (int i = 0; i < n; ++i) {
            if (i != k) {
                double factor = M[i * (2 * n) + k];
                for (int j = k; j < 2 * n; ++j) {
                    M[i * (2 * n) + j] -= factor * M[k * (2 * n) + j];
                }
            }
        }
    }

    // 提取逆矩阵
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A_inv[i * n + j] = M[i * (2 * n) + (j + n)];
        }
    }
    return true;
}

// ============================================================================
// 矩阵相关算法
// ============================================================================

/**
 * @brief 累加最小二乘Hessian矩阵: H += J^T * J, g += J^T * e
 * @param J 雅可比矩阵 (3×6)
 * @param meas 测量值 (3×1)
 * @param H Hessian矩阵 (6×6)
 * @param g 梯度向量 (6×1)
 */
inline auto accumulate_least_squares(const double* J, const double* meas, double* H, double* g) -> void {
    // H += J^T * J
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            double val = 0;
            for (int k = 0; k < 3; ++k) {
                val += J[k * 6 + r] * J[k * 6 + c];
            }
            H[r * 6 + c] += val;
        }
    }

    // g += J^T * meas
    for (int r = 0; r < 6; ++r) {
        double val = 0;
        for (int k = 0; k < 3; ++k) {
            val += J[k * 6 + r] * meas[k];
        }
        g[r] += val;
    }
}

/**
 * @brief 高效 3x3 SVD 分解 (固定步数 Jacobi): A = U * diag(S) * V^T
 * @param A 输入矩阵 [9]
 * @param U 输出左奇异向量 [9]
 * @param S 输出奇异值 [3] (从大到小排序)
 * @param V 输出右奇异向量 [9]
 * @return true 总是返回 true (固定步数必定完成)
 */
auto svd3x3(const double* A, double* U, double* S, double* V) -> bool ;

/**
 * @brief 通用矩阵 SVD (单边 Jacobi): A = U * diag(S) * V^T
 * @details 适用于行数 >= 列数的情况 (M >= N). 零动态内存分配.
 * 计算后 U 包含左奇异向量, A 的内容会被破坏(如果 U!=A).
 * @param A 输入矩阵 [rows * cols].
 * @param rows 行数 M
 * @param cols 列数 N (要求 M >= N)
 * @param U 输出矩阵 [rows * cols]. 传入时若 U != A, 会先将 A 拷贝到 U. 最终存储左奇异向量.
 * @param S 输出向量 [cols]. 存储奇异值.
 * @param V 输出矩阵 [cols * cols]. 存储右奇异向量.
 * @return true 收敛, false 超过最大迭代次数
 */
auto svd_general(const double* A, int rows, int cols, double* U, double* S, double* V) -> bool ;

/**
 * @brief 矩形矩阵奇异值 (仅计算奇异值，支持 rows < cols)
 * @param A 输入矩阵 [rows * cols]，行优先
 * @param rows 行数 M
 * @param cols 列数 N
 * @param S 输出奇异值 [min(rows,cols)]，从大到小
 * @return true 收敛, false 失败或未收敛
 */
auto svd_singular_values_rect(const double* A, int rows, int cols, double* S) -> bool ;

// ============================================================================
// 旋转矩阵生成函数：绕坐标轴旋转
// ============================================================================

/**
 * @brief 生成绕X轴旋转的3×3旋转矩阵
 * @param angle 旋转角度 (rad)
 * @param R 输出的3×3旋转矩阵 (行优先存储)
 *
 * @details 旋转矩阵形式：
 *   R_x(θ) = [1     0       0   ]
 *            [0   cos(θ) -sin(θ)]
 *            [0   sin(θ)  cos(θ)]
 */
inline auto rotx(double angle, double* R) -> void {
    double s, c;
    fast_sincos(angle, s, c);
    R[0] = 1.0;
    R[1] = 0.0;
    R[2] = 0.0;
    R[3] = 0.0;
    R[4] = c;
    R[5] = -s;
    R[6] = 0.0;
    R[7] = s;
    R[8] = c;
}

/**
 * @brief 生成绕Y轴旋转的3×3旋转矩阵
 * @param angle 旋转角度 (rad)
 * @param R 输出的3×3旋转矩阵 (行优先存储)
 *
 * @details 旋转矩阵形式：
 *   R_y(θ) = [ cos(θ)   0   sin(θ)]
 *            [   0      1     0   ]
 *            [-sin(θ)   0   cos(θ)]
 */
inline auto roty(double angle, double* R) -> void {
    double s, c;
    fast_sincos(angle, s, c);
    R[0] = c;
    R[1] = 0.0;
    R[2] = s;
    R[3] = 0.0;
    R[4] = 1.0;
    R[5] = 0.0;
    R[6] = -s;
    R[7] = 0.0;
    R[8] = c;
}

/**
 * @brief 生成绕Z轴旋转的3×3旋转矩阵
 * @param angle 旋转角度 (rad)
 * @param R 输出的3×3旋转矩阵 (行优先存储)
 *
 * @details 旋转矩阵形式：
 *   R_z(θ) = [cos(θ) -sin(θ)   0]
 *            [sin(θ)  cos(θ)   0]
 *            [  0       0      1]
 */
inline auto rotz(double angle, double* R) -> void {
    double s, c;
    fast_sincos(angle, s, c);
    R[0] = c;
    R[1] = -s;
    R[2] = 0.0;
    R[3] = s;
    R[4] = c;
    R[5] = 0.0;
    R[6] = 0.0;
    R[7] = 0.0;
    R[8] = 1.0;
}

/**
 * @brief 创建沿Z轴平移和旋转的变换矩阵
 * @param dz Z轴平移距离 (m)
 * @param angle_z 绕Z轴旋转角度 (rad)
 * @param T_out 输出的4×4变换矩阵
 */
inline auto create_transform_z(double dz, double angle_z, double* T_out) -> void {
    double s, c;
    fast_sincos(angle_z, s, c);

    T_out[0] = c;
    T_out[1] = -s;
    T_out[2] = 0;
    T_out[3] = 0;
    T_out[4] = s;
    T_out[5] = c;
    T_out[6] = 0;
    T_out[7] = 0;
    T_out[8] = 0;
    T_out[9] = 0;
    T_out[10] = 1;
    T_out[11] = dz;
    T_out[12] = 0;
    T_out[13] = 0;
    T_out[14] = 0;
    T_out[15] = 1;
}

// ============================================================================
// 四元数运算 (w, x, y, z)
// ============================================================================

/**
 * @brief 四元数归一化
 */
inline auto quat_normalize(double* q) -> void {
    double n = safe_sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > EPSILON) {
        q[0] /= n;
        q[1] /= n;
        q[2] /= n;
        q[3] /= n;
    } else {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    }
}

/**
 * @brief 四元数取负
 */
inline auto quat_neg(double* q) -> void {
    for (int i = 0; i < 4; i++) {
        q[i] = -q[i];
    }
    quat_normalize(q);
}

/**
 * @brief 四元数取负
 */
inline auto quat_neg(const double* q, double* out) -> void {
    for (int i = 0; i < 4; i++) {
        out[i] = -q[i];
    }
    quat_normalize(out);
}

/**
 * @brief 四元数乘法 out = q1 * q2
 */
inline auto quat_mul(const double* q1, const double* q2, double* out) -> void {
    double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    out[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    out[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    out[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    out[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    quat_normalize(out);
}

/**
 * @brief 四元数共轭 out = q*
 */
inline auto quat_conj(const double* q, double* out) -> void {
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
    quat_normalize(out);
}

/**
 * @brief 四元数逆 out = q^-1 (假设q已归一化)
 */
inline auto quat_inv(const double* q, double* out) -> void {
    quat_conj(q, out);
    quat_normalize(out);
}

// ============================================================================
// 位姿表示形式的各种转换
// ============================================================================

/**
 * @brief 罗德里格斯公式: 绕单位轴k旋转theta角度
 */
inline auto rodrigues(const double* RTB_RESTRICT k, double theta, double* RTB_RESTRICT R) -> void {
    double s, c;
    fast_sincos(theta, s, c);
    double v = 1.0 - c;

    R[0] = k[0] * k[0] * v + c;
    R[1] = k[0] * k[1] * v - k[2] * s;
    R[2] = k[0] * k[2] * v + k[1] * s;
    R[3] = k[1] * k[0] * v + k[2] * s;
    R[4] = k[1] * k[1] * v + c;
    R[5] = k[1] * k[2] * v - k[0] * s;
    R[6] = k[2] * k[0] * v - k[1] * s;
    R[7] = k[2] * k[1] * v + k[0] * s;
    R[8] = k[2] * k[2] * v + c;
}

/**
 * @brief 3×3反对称矩阵构造 (skew-symmetric matrix)
 * [v]_× = [  0  -v3   v2 ]
 *         [ v3    0  -v1 ]
 *         [-v2   v1    0 ]
 */
constexpr inline auto vec3_to_skew_symmetric(const double* v, double* S) -> void {
    S[0] = 0.0;
    S[1] = -v[2];
    S[2] = v[1];
    S[3] = v[2];
    S[4] = 0.0;
    S[5] = -v[0];
    S[6] = -v[1];
    S[7] = v[0];
    S[8] = 0.0;
}

/**
 * @brief 四元数转3×3旋转矩阵
 */
inline auto rq2rm(const double* RTB_RESTRICT q, double* RTB_RESTRICT R) -> void {
    double tmp_q[4];
    std::memcpy(tmp_q, q, 4 * sizeof(double));
    quat_normalize(tmp_q);
    double w = tmp_q[0], x = tmp_q[1], y = tmp_q[2], z = tmp_q[3];
    double xx = x * x, yy = y * y, zz = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    R[0] = 1.0 - 2.0 * (yy + zz);
    R[1] = 2.0 * (xy - wz);
    R[2] = 2.0 * (xz + wy);
    R[3] = 2.0 * (xy + wz);
    R[4] = 1.0 - 2.0 * (xx + zz);
    R[5] = 2.0 * (yz - wx);
    R[6] = 2.0 * (xz - wy);
    R[7] = 2.0 * (yz + wx);
    R[8] = 1.0 - 2.0 * (xx + yy);
}

/**
 * @brief 四元数转4×4齐次矩阵（仅旋转部分）
 */
inline auto rq2pm(const double* RTB_RESTRICT q, double* RTB_RESTRICT pm) -> void {
    double tmp_q[4];
    std::memcpy(tmp_q, q, 4 * sizeof(double));
    quat_normalize(tmp_q);
    double w = tmp_q[0], x = tmp_q[1], y = tmp_q[2], z = tmp_q[3];
    double xx = x * x, yy = y * y, zz = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    pm[0] = 1.0 - 2.0 * (yy + zz);
    pm[1] = 2.0 * (xy - wz);
    pm[2] = 2.0 * (xz + wy);
    pm[4] = 2.0 * (xy + wz);
    pm[5] = 1.0 - 2.0 * (xx + zz);
    pm[6] = 2.0 * (yz - wx);
    pm[8] = 2.0 * (xz - wy);
    pm[9] = 2.0 * (yz + wx);
    pm[10] = 1.0 - 2.0 * (xx + yy);
}

/**
 * @brief 3×3旋转矩阵转四元数
 */
inline auto rm2rq(const double* RTB_RESTRICT R, double* RTB_RESTRICT q) -> void {
    double tr = R[0] + R[4] + R[8];
    if (tr > 0) {
        double S = safe_sqrt(tr + 1.0) * 2;
        q[0] = 0.25 * S;
        q[1] = (R[7] - R[5]) / S;
        q[2] = (R[2] - R[6]) / S;
        q[3] = (R[3] - R[1]) / S;
    } else if ((R[0] > R[4]) && (R[0] > R[8])) {
        double S = safe_sqrt(1.0 + R[0] - R[4] - R[8]) * 2;
        q[0] = (R[7] - R[5]) / S;
        q[1] = 0.25 * S;
        q[2] = (R[1] + R[3]) / S;
        q[3] = (R[2] + R[6]) / S;
    } else if (R[4] > R[8]) {
        double S = safe_sqrt(1.0 + R[4] - R[0] - R[8]) * 2;
        q[0] = (R[2] - R[6]) / S;
        q[1] = (R[1] + R[3]) / S;
        q[2] = 0.25 * S;
        q[3] = (R[5] + R[7]) / S;
    } else {
        double S = safe_sqrt(1.0 + R[8] - R[0] - R[4]) * 2;
        q[0] = (R[3] - R[1]) / S;
        q[1] = (R[2] + R[6]) / S;
        q[2] = (R[5] + R[7]) / S;
        q[3] = 0.25 * S;
    }
    quat_normalize(q);
}

/**
 * @brief 欧拉角(ZYX)转旋转矩阵
 */
inline auto re2rm(double r, double p, double y, double* R) -> void {
    double cx = cos(r), sx = sin(r);
    double cy = cos(p), sy = sin(p);
    double cz = cos(y), sz = sin(y);
    // Row major
    R[0] = cz * cy;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;
    R[3] = sz * cy;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;
    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;
}

/**
 * @brief 4×4齐次矩阵转四元数
 */
inline auto pm2rq(const double* RTB_RESTRICT pm, double* RTB_RESTRICT q) -> void {
    double R[9];
    pm2rm(pm, R);
    rm2rq(R, q);
}

/**
 * @brief 位置+四元数(pq)转4×4齐次矩阵(pm)
 * @param pq 输入：[x, y, z, qw, qx, qy, qz] (7维)
 * @param pm 输出：4×4齐次矩阵 (16维, row-major)
 * @note pq格式：前3维位置，后4维四元数(w在前)
 */
inline auto pq2pm(const double* RTB_RESTRICT pq, double* RTB_RESTRICT pm) -> void {
    // 提取位置和四元数
    double pos[3] = {pq[0], pq[1], pq[2]};
    double quat[4] = {pq[3], pq[4], pq[5], pq[6]};  // [qw, qx, qy, qz]

    // 四元数转旋转矩阵 (3x3)
    double R[9];
    rq2rm(quat, R);

    // 组装4x4齐次矩阵
    pm[0] = R[0];
    pm[1] = R[1];
    pm[2] = R[2];
    pm[3] = pos[0];
    pm[4] = R[3];
    pm[5] = R[4];
    pm[6] = R[5];
    pm[7] = pos[1];
    pm[8] = R[6];
    pm[9] = R[7];
    pm[10] = R[8];
    pm[11] = pos[2];
    pm[12] = 0.0;
    pm[13] = 0.0;
    pm[14] = 0.0;
    pm[15] = 1.0;
}

/**
 * @brief 4×4齐次矩阵(pm)转位置+四元数(pq)
 * @param pm 输入：4×4齐次矩阵 (16维, row-major)
 * @param pq 输出：[x, y, z, qw, qx, qy, qz] (7维)
 * @note pq格式：前3维位置，后4维四元数(w在前)
 */
inline auto pm2pq(const double* RTB_RESTRICT pm, double* RTB_RESTRICT pq) -> void {
    // 提取位置
    pq[0] = pm[3];
    pq[1] = pm[7];
    pq[2] = pm[11];

    // 提取旋转矩阵 (3x3)
    double R[9] = {pm[0], pm[1], pm[2], pm[4], pm[5], pm[6], pm[8], pm[9], pm[10]};

    // 旋转矩阵转四元数
    double quat[4];
    rm2rq(R, quat);  // [qw, qx, qy, qz]

    pq[3] = quat[0];  // qw
    pq[4] = quat[1];  // qx
    pq[5] = quat[2];  // qy
    pq[6] = quat[3];  // qz
}

/**
 * @brief 从4×4齐次矩阵提取位置向量
 */
constexpr inline auto mat4_get_pos(const double* RTB_RESTRICT pm, double* RTB_RESTRICT pos) -> void {
    pos[0] = pm[3];
    pos[1] = pm[7];
    pos[2] = pm[11];
}

/**
 * @brief 设置4×4齐次矩阵的位置
 */
inline auto mat4_set_pos(double* pm, const double* pos) -> void {
    pm[3] = pos[0];
    pm[7] = pos[1];
    pm[11] = pos[2];
}

/**
 * @brief 由3×3旋转矩阵和3×1平移向量组合成4×4齐次变换矩阵
 * @param R 输入的3×3旋转矩阵 (行优先存储)
 * @param t 输入的3×1平移向量 [x, y, z]
 * @param T 输出的4×4齐次变换矩阵 (行优先存储)
 *
 * @details 组合形式：
 *   T = [R  t]
 *       [0  1]
 */
constexpr inline auto compose_transform(
    const double* RTB_RESTRICT R, const double* RTB_RESTRICT t, double* RTB_RESTRICT T) -> void {
    // 复制旋转矩阵部分
    T[0] = R[0];
    T[1] = R[1];
    T[2] = R[2];
    T[4] = R[3];
    T[5] = R[4];
    T[6] = R[5];
    T[8] = R[6];
    T[9] = R[7];
    T[10] = R[8];

    // 设置平移部分
    T[3] = t[0];
    T[7] = t[1];
    T[11] = t[2];

    // 设置齐次部分
    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;
}

/**
 * @brief 从4×4齐次变换矩阵分解出3×3旋转矩阵和3×1平移向量
 * @param T 输入的4×4齐次变换矩阵 (行优先存储)
 * @param R 输出的3×3旋转矩阵 (行优先存储)
 * @param t 输出的3×1平移向量 [x, y, z]
 *
 * @details 分解形式：
 *   T = [R  t]  → 提取 R 和 t
 *       [0  1]
 */
inline auto decompose_transform(const double* T, double* R = nullptr, double* t = nullptr) -> void {
    // 提取旋转矩阵部分
    if (R) {
        R[0] = T[0];
        R[1] = T[1];
        R[2] = T[2];
        R[3] = T[4];
        R[4] = T[5];
        R[5] = T[6];
        R[6] = T[8];
        R[7] = T[9];
        R[8] = T[10];
    }

    // 提取平移向量
    if (t) {
        t[0] = T[3];
        t[1] = T[7];
        t[2] = T[11];
    }
}

/**
 * @brief 由旋转矩阵生成4×4齐次变换矩阵（平移为零）
 * @param R 输入的3×3旋转矩阵 (行优先存储)
 * @param T 输出的4×4齐次变换矩阵 (行优先存储)
 */
constexpr inline auto rm2pm(const double* RTB_RESTRICT R, double* RTB_RESTRICT T) -> void {
    double zero_pos[3] = {0.0, 0.0, 0.0};
    compose_transform(R, zero_pos, T);
}

/**
 * @brief 由平移向量生成4×4齐次变换矩阵（旋转为单位矩阵）
 * @param t 输入的3×1平移向量 [x, y, z]
 * @param T 输出的4×4齐次变换矩阵 (行优先存储)
 */
inline auto pp2pm(const double* RTB_RESTRICT t, double* RTB_RESTRICT T) -> void {
    double R_eye[9];
    mat3_identity(R_eye);
    compose_transform(R_eye, t, T);
}

/**
 * @brief 轴角转四元数 (axis: 归一化轴, angle: 弧度)
 * @details 小角度时 (s < 1e-6) 使用近似 q = [cos(angle/2), sin(angle/2) * axis]，与极限 angle/sin(angle/2) -> 2 一致
 * @param axis 轴向量 [rx, ry, rz]
 * @param angle 旋转角 (rad)
 * @param q 输出四元数 [qw, qx, qy, qz]
 */
inline auto ra2rq(const double* RTB_RESTRICT axis, double angle, double* RTB_RESTRICT q) -> void {
    double half = angle * 0.5;
    double s, c;
    fast_sincos(half, s, c);
    double n = vec3_norm(axis);
    if (n < EPSILON) {
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    } else {
        q[0] = c;
        q[1] = (axis[0] / n) * s;
        q[2] = (axis[1] / n) * s;
        q[3] = (axis[2] / n) * s;
    }
    // 显式归一化以确保数值稳定性
    quat_normalize(q);
}

/**
 * @brief 四元数转轴角向量 (angle * axis)
 * @param q 四元数 [w, x, y, z]
 * @param v 输出轴角向量 [rx, ry, rz]，模长 = 旋转角 (rad)
 * @details 小角度时 (s < 1e-6) 使用近似 v = 2*q_vec，与极限 angle/sin(angle/2) -> 2 一致
 */
inline auto rq2ra(const double* RTB_RESTRICT q, double* RTB_RESTRICT v) -> void {
    constexpr double RQ2RA_SMALL_S = 1e-6;  // 小角度阈值 sin(angle/2)
    double w = (q[0] > 1.0) ? 1.0 : (q[0] < -1.0 ? -1.0 : q[0]);
    double angle = 2.0 * safe_acos(w);
    double s = safe_sqrt(1.0 - w * w);
    if (s < RQ2RA_SMALL_S) {
        // 小角度近似: angle/sin(angle/2) -> 2
        v[0] = q[1] * 2.0;
        v[1] = q[2] * 2.0;
        v[2] = q[3] * 2.0;
    } else {
        double factor = angle / s;
        v[0] = q[1] * factor;
        v[1] = q[2] * factor;
        v[2] = q[3] * factor;
    }
}

/**
 * @brief 4×4矩阵转位姿向量 (ZYX欧拉角)
 * @param T 4×4齐次变换矩阵
 * @param pose 输出位姿向量 [x, y, z, rz, ry, rx] (6维数组)
 */
inline auto pm2pe(const double* RTB_RESTRICT T, double* RTB_RESTRICT pose) -> void {
    double x = T[3], y = T[7], z = T[11];
    double r11 = T[0], r21 = T[4], r31 = T[8];
    double r32 = T[9], r33 = T[10];

    double sy = safe_sqrt(r11 * r11 + r21 * r21);
    bool singular = sy < EPSILON;
    double rz, ry, rx;

    if (!singular) {
        rz = std::atan2(r21, r11);
        ry = std::atan2(-r31, sy);
        rx = std::atan2(r32, r33);
    } else {
        rz = 0;
        if (r31 < 0) {
            ry = PI / 2.0;
            rx = std::atan2(T[1], T[2]);
        } else {
            ry = -PI / 2.0;
            rx = std::atan2(-T[1], -T[2]);
        }
    }

    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    pose[3] = rz;
    pose[4] = ry;
    pose[5] = rx;
}

/**
 * @brief 4×4矩阵转位姿向量 (ZYX欧拉角) - 返回std::array版本
 * @param T 4×4齐次变换矩阵数组
 * @return 位姿向量 [x, y, z, rz, ry, rx]
 */
inline auto pm2pe(const std::array<double, 16>& T) -> std::array<double, 6> {
    std::array<double, 6> pose;
    pm2pe(T.data(), pose.data());
    return pose;
}

/**
 * @brief 位姿向量转4×4齐次变换矩阵
 * @param pose 位姿向量 [x, y, z, rz, ry, rx]，长度至少 6
 * @param T_out 输出 4×4 齐次变换矩阵（行优先），长度至少 16
 */
inline auto pe2pm(const double* RTB_RESTRICT pe, double* RTB_RESTRICT T_out) -> void {
    double x = pe[0], y = pe[1], z = pe[2];
    double rz = pe[3], ry = pe[4], rx = pe[5];

    double cz = std::cos(rz), sz = std::sin(rz);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cx = std::cos(rx), sx = std::sin(rx);

    T_out[0] = cz * cy;
    T_out[1] = cz * sy * sx - sz * cx;
    T_out[2] = cz * sy * cx + sz * sx;
    T_out[3] = x;

    T_out[4] = sz * cy;
    T_out[5] = sz * sy * sx + cz * cx;
    T_out[6] = sz * sy * cx - cz * sx;
    T_out[7] = y;

    T_out[8] = -sy;
    T_out[9] = cy * sx;
    T_out[10] = cy * cx;
    T_out[11] = z;

    T_out[12] = 0.0;
    T_out[13] = 0.0;
    T_out[14] = 0.0;
    T_out[15] = 1.0;
}

/**
 * @brief 位置+欧拉角(pe)转位置+四元数(pq)
 * @param pe 输入：[x, y, z, rz, ry, rx]，ZYX 内旋欧拉角，与 pe2pm / pm2pe 一致
 * @param pq 输出：[x, y, z, qw, qx, qy, qz]，长度至少 7；四元数与 pq2pm / pm2pq 一致 (w, x, y, z)
 */
inline auto pe2pq(const double* RTB_RESTRICT pe, double* RTB_RESTRICT pq) -> void {
    pq[0] = pe[0];
    pq[1] = pe[1];
    pq[2] = pe[2];

    const double rz = pe[3], ry = pe[4], rx = pe[5];
    double sz, cz;
    double sy, cy;
    double sx, cx;
    fast_sincos(rz, sz, cz);
    fast_sincos(ry, sy, cy);
    fast_sincos(rx, sx, cx);

    double R[9];
    R[0] = cz * cy;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;
    R[3] = sz * cy;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;
    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;

    rm2rq(R, pq + 3);
}

// ============================================================================
// 插值工具
// ============================================================================

/**
 * @brief 向量线性插值
 */
constexpr inline auto vec_lerp(const double* v0, const double* v1, double t, double* out, int n) -> void {
    for (int i = 0; i < n; ++i) {
        out[i] = v0[i] + (v1[i] - v0[i]) * t;
    }
}

/**
 * @brief 3D向量线性插值特化
 */
constexpr inline auto vec3_lerp(const double* v0, const double* v1, double t, double* out) -> void {
    out[0] = v0[0] + (v1[0] - v0[0]) * t;
    out[1] = v0[1] + (v1[1] - v0[1]) * t;
    out[2] = v0[2] + (v1[2] - v0[2]) * t;
}

/**
 * @brief 四元数球面线性插值 (Slerp)
 */
inline auto quat_slerp(const double* q1, const double* q2, double t, double* q_out) -> void {
    double qa[4], qb[4];
    vec_copy(q1, qa, 4);
    quat_normalize(qa);
    vec_copy(q2, qb, 4);
    quat_normalize(qb);

    double dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];

    // 选择最短路径
    if (dot < 0.0) {
        qb[0] = -qb[0];
        qb[1] = -qb[1];
        qb[2] = -qb[2];
        qb[3] = -qb[3];
        dot = -dot;
    }

    // 接近时使用线性插值
    if (dot > 0.9995) {
        for (int i = 0; i < 4; i++)
            q_out[i] = qa[i] + t * (qb[i] - qa[i]);
        quat_normalize(q_out);
        return;
    }

    double theta_0 = safe_acos(dot);
    double theta = theta_0 * t;
    double s0 = std::cos(theta) - dot * std::sin(theta) / std::sin(theta_0);
    double s1 = std::sin(theta) / std::sin(theta_0);

    for (int i = 0; i < 4; i++)
        q_out[i] = s0 * qa[i] + s1 * qb[i];
    quat_normalize(q_out);
}

/**
 * @brief 位姿矩阵线性插值 (位置线性插值 + 姿态Slerp插值)
 */
inline auto mat4_interpolate(const double* T_start, const double* T_end, double t, double* T_out) -> void {
    // 位置插值
    double pos_start[3], pos_end[3], pos_out[3];
    mat4_get_pos(T_start, pos_start);
    mat4_get_pos(T_end, pos_end);
    vec3_lerp(pos_start, pos_end, t, pos_out);

    // 姿态插值
    double q_start[4], q_end[4], q_out[4];
    pm2rq(T_start, q_start);
    pm2rq(T_end, q_end);
    quat_slerp(q_start, q_end, t, q_out);

    // 组合
    rq2pm(q_out, T_out);
    mat4_set_pos(T_out, pos_out);

    // 设置齐次部分
    T_out[12] = 0.0;
    T_out[13] = 0.0;
    T_out[14] = 0.0;
    T_out[15] = 1.0;
}

/**
 * @brief 生成线性插值轨迹点
 * @param T_start 起始位姿 (4×4 行优先)，长度至少 16
 * @param T_end 结束位姿 (4×4 行优先)，长度至少 16
 * @param N 总点数
 * @param k 当前点索引 (0 <= k <= N)
 * @param T_out 输出插值后的位姿 (4×4)，长度至少 16
 */
inline auto generateLinearPath(const double* T_start, const double* T_end, int N, int k, double* T_out) -> void {
    double t = (N > 0) ? (double)k / N : 0.0;
    mat4_interpolate(T_start, T_end, t, T_out);
}

// ============================================================================
// 旋量代数
// ============================================================================

/**
 * @brief 计算反对称矩阵 (Skew-symmetric matrix)
 * @param v 输入 3维向量
 * @param skew 输出 3x3 矩阵
 */
constexpr inline auto vec3_skew(const double* v, double* skew) -> void {
    skew[0] = 0;
    skew[1] = -v[2];
    skew[2] = v[1];
    skew[3] = v[2];
    skew[4] = 0;
    skew[5] = -v[0];
    skew[6] = -v[1];
    skew[7] = v[0];
    skew[8] = 0;
}

/**
 * @brief 简单的 6维向量加法
 */
constexpr inline auto vec6_add(const double* a, const double* b, double* out) -> void {
    for (int i = 0; i < 6; ++i)
        out[i] = a[i] + b[i];
}

/**
 * @brief 简单的 6维向量减法
 */
constexpr inline auto vec6_sub(const double* a, const double* b, double* out) -> void {
    for (int i = 0; i < 6; ++i)
        out[i] = a[i] - b[i];
}

/**
 * @brief 简单的 6维向量数乘
 */
constexpr inline auto vec6_scale(const double* a, double s, double* out) -> void {
    for (int i = 0; i < 6; ++i)
        out[i] = a[i] * s;
}

// 鲁棒的 so(3) 对数映射 (Rotation Matrix -> Omega Vector)
inline auto mat3_log_so3(const double* RTB_RESTRICT R, double* RTB_RESTRICT w) -> void {
    double tr = R[0] + R[4] + R[8];
    double cos_theta = 0.5 * (tr - 1.0);
    // Clamp to [-1, 1] to avoid NaN
    if (cos_theta > 1.0)
        cos_theta = 1.0;
    if (cos_theta < -1.0)
        cos_theta = -1.0;

    double theta = safe_acos(cos_theta);

    if (theta < 1e-6) {
        // Taylor expansion for small angle: sin(x)/x -> 1
        // w = 0.5 * (R - R^T)
        w[0] = 0.5 * (R[7] - R[5]);
        w[1] = 0.5 * (R[2] - R[6]);
        w[2] = 0.5 * (R[3] - R[1]);
    } else {
        double factor = theta / (2.0 * std::sin(theta));
        w[0] = factor * (R[7] - R[5]);
        w[1] = factor * (R[2] - R[6]);
        w[2] = factor * (R[3] - R[1]);
    }
}

// 鲁棒的 SE(3) 对数映射 (T -> Twist)
// 计算 Twist 使得 T = exp(Twist)
// Twist = [v, w], v is linear, w is angular
inline auto mat4_log_se3(const double* RTB_RESTRICT T, double* RTB_RESTRICT twist) -> void {
    double R[9];
    // Extract R
    R[0] = T[0];
    R[1] = T[1];
    R[2] = T[2];
    R[3] = T[4];
    R[4] = T[5];
    R[5] = T[6];
    R[6] = T[8];
    R[7] = T[9];
    R[8] = T[10];
    double p[3] = {T[3], T[7], T[11]};

    // 1. Compute w from R
    double w[3];
    mat3_log_so3(R, w);  // Use robust so3 log

    twist[3] = w[0];
    twist[4] = w[1];
    twist[5] = w[2];

    double theta = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);

    // 2. Compute Left Jacobian Inverse * p
    // V_inv = I - 0.5*w_skew + coef * w_skew^2
    double coef;

    // 【核心修复】：将阈值提高到 1e-3，彻底规避 (1 - cos(theta)) 引发的灾难性精度抵消
    // 当 theta < 1e-3 时，使用泰勒级数展开的二阶截断，保证平滑且计算效率极高
    if (theta < 1e-3) {
        coef = 1.0 / 12.0 + (theta * theta) / 720.0;
    } else {
        // 使用标准的解析解
        coef = (1.0 - (theta * std::sin(theta)) / (2.0 * (1.0 - std::cos(theta)))) / (theta * theta);
    }

    // w_skew
    double wx = w[0], wy = w[1], wz = w[2];

    // V_inv * p computation (unrolled for performance)
    // t = -0.5 * (w x p)
    double t[3];
    t[0] = -0.5 * (wy * p[2] - wz * p[1]);
    t[1] = -0.5 * (wz * p[0] - wx * p[2]);
    t[2] = -0.5 * (wx * p[1] - wy * p[0]);

    // t += coef * (w x (w x p))
    double wp[3];  // w x p
    wp[0] = wy * p[2] - wz * p[1];
    wp[1] = wz * p[0] - wx * p[2];
    wp[2] = wx * p[1] - wy * p[0];

    double wwp[3];  // w x wp
    wwp[0] = wy * wp[2] - wz * wp[1];
    wwp[1] = wz * wp[0] - wx * wp[2];
    wwp[2] = wx * wp[1] - wy * wp[0];

    // v = p + t + coef * wwp
    twist[0] = p[0] + t[0] + coef * wwp[0];
    twist[1] = p[1] + t[1] + coef * wwp[1];
    twist[2] = p[2] + t[2] + coef * wwp[2];
}

// 通过两个位姿差分计算 Spatial Twist (Base Frame)
// Twist = Log(T_next * T_curr^-1) / dt
inline auto calc_twist_from_pose_diff(const double* T_curr, const double* T_next, double dt, double* out_twist) -> void {
    double T_curr_inv[16];
    // Invert T_curr
    // R^T
    T_curr_inv[0] = T_curr[0];
    T_curr_inv[1] = T_curr[4];
    T_curr_inv[2] = T_curr[8];
    T_curr_inv[3] = 0;
    T_curr_inv[4] = T_curr[1];
    T_curr_inv[5] = T_curr[5];
    T_curr_inv[6] = T_curr[9];
    T_curr_inv[7] = 0;
    T_curr_inv[8] = T_curr[2];
    T_curr_inv[9] = T_curr[6];
    T_curr_inv[10] = T_curr[10];
    T_curr_inv[11] = 0;
    // -R^T * p
    double p[3] = {T_curr[3], T_curr[7], T_curr[11]};
    T_curr_inv[3] = -(T_curr_inv[0] * p[0] + T_curr_inv[1] * p[1] + T_curr_inv[2] * p[2]);
    T_curr_inv[7] = -(T_curr_inv[4] * p[0] + T_curr_inv[5] * p[1] + T_curr_inv[6] * p[2]);
    T_curr_inv[11] = -(T_curr_inv[8] * p[0] + T_curr_inv[9] * p[1] + T_curr_inv[10] * p[2]);
    T_curr_inv[12] = 0;
    T_curr_inv[13] = 0;
    T_curr_inv[14] = 0;
    T_curr_inv[15] = 1;

    double T_diff[16];
    // T_diff = T_next * T_curr_inv
    // (Standard mat4 mul)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T_diff[i * 4 + j] = T_next[i * 4 + 0] * T_curr_inv[0 * 4 + j] + T_next[i * 4 + 1] * T_curr_inv[1 * 4 + j] +
                                T_next[i * 4 + 2] * T_curr_inv[2 * 4 + j] + T_next[i * 4 + 3] * T_curr_inv[3 * 4 + j];
        }
    }

    double twist[6];
    mat4_log_se3(T_diff, twist);

    for (int i = 0; i < 6; ++i)
        out_twist[i] = twist[i] / dt;
}

// ============================================================================
// 力螺旋（Wrench）转换：力+力矩的坐标变换
// ============================================================================

/**
 * @brief 将力螺旋从坐标系A转换到坐标系B
 * @param R_ba 从A到B的旋转矩阵 (3x3)
 * @param r_ba A原点相对于B原点的位置向量（在B坐标系下表示）
 * @param wrench_a 在坐标系A下的力螺旋 [Fx, Fy, Fz, Mx, My, Mz]
 * @param wrench_b 输出：在坐标系B下的力螺旋
 *
 * @details 力螺旋转换公式：
 *   F_b = R * F_a
 *   M_b = R * M_a + r_ba × F_b
 */
inline auto transform_wrench(const double* R_ba, const double* r_ba, const double* wrench_a, double* wrench_b) -> void {
    // 1. 转换力：F_b = R * F_a
    mat3_vec_mul(R_ba, wrench_a, wrench_b);

    // 2. 转换力矩：M_b = R * M_a + r_ba × F_b
    double M_rotated[3];
    mat3_vec_mul(R_ba, wrench_a + 3, M_rotated);

    double r_cross_F[3];
    vec3_cross(r_ba, wrench_b, r_cross_F);  // 注意：使用转换后的F_b

    vec3_add(M_rotated, r_cross_F, wrench_b + 3);
}

/**
 * @brief 将力螺旋转换到不同作用点（同一坐标系）
 * @param r_new_old 新作用点相对于旧作用点的位置向量
 * @param wrench_old 在旧作用点测量的力螺旋
 * @param wrench_new 输出：换算到新作用点的力螺旋
 *
 * @details 力不变，力矩变化：
 *   F_new = F_old
 *   M_new = M_old + r_new_old × F_old
 */
inline auto change_wrench_point(const double* r_new_old, const double* wrench_old, double* wrench_new) -> void {
    // 力保持不变
    vec_copy(wrench_old, wrench_new, 3);

    // 力矩变换
    double r_cross_F[3];
    vec3_cross(r_new_old, wrench_old, r_cross_F);
    vec3_add(wrench_old + 3, r_cross_F, wrench_new + 3);
}

/**
 * @brief 将基坐标系下的力螺旋转换到末端坐标系
 * @param pm_ee 末端位姿 (4x4矩阵)
 * @param wrench_base 基坐标系下的力螺旋
 * @param wrench_ee 输出：末端坐标系下的力螺旋
 *
 * @details 用于将传感器测量的外力（已转到基坐标）转换回末端坐标系，
 *          这样导纳控制器可以在末端坐标系下进行控制
 */
inline auto wrench_base_to_ee(const double* pm_ee, const double* wrench_base, double* wrench_ee) -> void {
    // 提取旋转矩阵 R_base_ee
    double R[9];
    pm2rm(pm_ee, R);

    // 提取位置向量（末端相对于基坐标）
    double p_ee[3] = {pm_ee[3], pm_ee[7], pm_ee[11]};

    // 转换到末端坐标系：需要 R_ee_base = R^T
    double Rt[9];
    mat3_transpose(R, Rt);

    // F_ee = R^T * F_base
    mat3_vec_mul(Rt, wrench_base, wrench_ee);

    // M_ee = R^T * (M_base - p_ee × F_base)
    double p_cross_F[3];
    vec3_cross(p_ee, wrench_base, p_cross_F);

    double M_adjusted[3];
    vec3_sub(wrench_base + 3, p_cross_F, M_adjusted);

    mat3_vec_mul(Rt, M_adjusted, wrench_ee + 3);
}

/**
 * @brief 使用4×4齐次变换矩阵将力螺旋从坐标系A转换到坐标系B
 * @param T_ba 从A到B的4×4齐次变换矩阵
 * @param wrench_a 在坐标系A下的力螺旋 [Fx, Fy, Fz, Mx, My, Mz]
 * @param wrench_b 输出：在坐标系B下的力螺旋
 *
 * @details 这是 transform_wrench 的便捷版本，直接从变换矩阵提取旋转和位置
 */
inline auto transform_wrench_mat4(const double* T_ba, const double* wrench_a, double* wrench_b) -> void {
    // 提取旋转矩阵
    double R[9];
    pm2rm(T_ba, R);

    // 提取位置向量
    double r[3] = {T_ba[3], T_ba[7], T_ba[11]};

    // 调用通用转换函数
    transform_wrench(R, r, wrench_a, wrench_b);
}

// ============================================================================
// 速度螺旋（Velocity Twist）转换：线速度+角速度的坐标变换
// ============================================================================

/**
 * @brief 将速度螺旋从坐标系A转换到坐标系B
 * @param R_ba 从A到B的旋转矩阵 (3x3)
 * @param r_ba A原点相对于B原点的位置向量（在B坐标系下表示）
 * @param twist_a 在坐标系A下的速度螺旋 [vx, vy, vz, wx, wy, wz]
 * @param twist_b 输出：在坐标系B下的速度螺旋
 *
 * @details 速度螺旋转换公式（与力螺旋不同！）：
 *   v_b = R * v_a + w_b × r_ba
 *   w_b = R * w_a
 *
 * 注意：速度螺旋的转换与力螺旋不同，因为线速度包含角速度产生的项
 */
inline auto transform_twist(const double* R_ba, const double* r_ba, const double* twist_a, double* twist_b) -> void {
    // 1. 先转换角速度：w_b = R * w_a
    mat3_vec_mul(R_ba, twist_a + 3, twist_b + 3);

    // 2. 转换线速度：v_b = R * v_a + w_b × r_ba
    double v_rotated[3];
    mat3_vec_mul(R_ba, twist_a, v_rotated);

    double w_cross_r[3];
    vec3_cross(twist_b + 3, r_ba, w_cross_r);  // 注意：使用转换后的w_b

    vec3_add(v_rotated, w_cross_r, twist_b);
}

/**
 * @brief 将速度螺旋转换到不同作用点（同一坐标系）
 * @param r_new_old 新作用点相对于旧作用点的位置向量
 * @param twist_old 在旧作用点的速度螺旋 [vx, vy, vz, wx, wy, wz]
 * @param twist_new 输出：在新作用点的速度螺旋
 *
 * @details 速度螺旋作用点转换公式：
 *   v_new = v_old + w × r_new_old
 *   w_new = w_old  （角速度不变）
 */
inline auto change_twist_point(const double* r_new_old, const double* twist_old, double* twist_new) -> void {
    // 角速度保持不变
    vec_copy(twist_old + 3, twist_new + 3, 3);

    // 速度进行补偿
    double w_cross_r[3];
    vec3_cross(r_new_old, twist_old + 3, w_cross_r);
    vec3_add(twist_old, w_cross_r, twist_new);
}

/**
 * @brief 使用4×4齐次变换矩阵将速度螺旋从坐标系A转换到坐标系B
 * @param T_ba 从A到B的4×4齐次变换矩阵
 * @param twist_a 在坐标系A下的速度螺旋 [vx, vy, vz, wx, wy, wz]
 * @param twist_b 输出：在坐标系B下的速度螺旋
 *
 * @details 这是 transform_twist 的便捷版本，直接从变换矩阵提取旋转和位置
 */
inline auto transform_twist_mat4(const double* T_ba, const double* twist_a, double* twist_b) -> void {
    // 提取旋转矩阵
    double R[9];
    pm2rm(T_ba, R);

    // 提取位置向量
    double r[3] = {T_ba[3], T_ba[7], T_ba[11]};

    // 调用通用转换函数
    transform_twist(R, r, twist_a, twist_b);
}

/**
 * @brief 将基坐标系下的速度螺旋转换到末端坐标系
 * @param pm_ee 末端位姿 (4x4矩阵)
 * @param twist_base 基坐标系下的速度螺旋 [vx, vy, vz, wx, wy, wz]
 * @param twist_ee 输出：末端坐标系下的速度螺旋
 *
 * @details 用于将速度螺旋从基坐标系转换到末端坐标系
 */
inline auto twist_base_to_ee(const double* pm_ee, const double* twist_base, double* twist_ee) -> void {
    // 提取旋转矩阵 R_base_ee
    double R[9];
    pm2rm(pm_ee, R);

    // 提取位置向量（末端相对于基坐标）
    double p_ee[3] = {pm_ee[3], pm_ee[7], pm_ee[11]};

    // 转换到末端坐标系：需要 R_ee_base = R^T
    double Rt[9];
    mat3_transpose(R, Rt);

    // 先转换角速度：w_ee = R^T * w_base
    mat3_vec_mul(Rt, twist_base + 3, twist_ee + 3);

    // 转换线速度：v_ee = R^T * (v_base - w_base × p_ee)
    double w_cross_p[3];
    vec3_cross(twist_base + 3, p_ee, w_cross_p);

    double v_adjusted[3];
    vec3_sub(twist_base, w_cross_p, v_adjusted);

    mat3_vec_mul(Rt, v_adjusted, twist_ee);
}

// ============================================================================
// 旋量理论核心算法 (Screw Theory & Lie Algebra)
// ============================================================================
// 约定：
// 1. 默认旋量顺序 (Twist/Wrench) 为 [Linear; Angular] 即 [v; w] 或 [f; tau]。
//    这是 ROS 和现代工程类教材（如 Lynch Modern Robotics）常用的顺序。
// 2. 如果需要 [Angular; Linear] 顺序，提供了 explicit 的转换函数。
// 3. 所有的输入输出数组长度均为 6 (double[6]) 或 16 (double[16] for T)。
// ============================================================================
/**
 * @brief 交换旋量顺序 [v; w] <-> [w; v]
 * @param src 输入 6维向量
 * @param dst 输出 6维向量
 */
constexpr inline auto swap_screw_order(const double* src, double* dst) -> void {
    // 暂存前三维，防止 src == dst 时数据覆盖
    double v[3] = {src[0], src[1], src[2]};
    dst[0] = src[3];
    dst[1] = src[4];
    dst[2] = src[5];
    dst[3] = v[0];
    dst[4] = v[1];
    dst[5] = v[2];
}

/**
 * @brief 计算 6x6 伴随矩阵 (Adjoint Matrix)
 * @details 用于将运动旋量(Twist)从坐标系 {B} 变换到 {A}
 * V_A = Ad_T * V_B
 * Ad_T = [ R    skew(p)*R ]
 * [ 0        R     ]
 * (基于 [v; w] 顺序)
 * @param T 输入 4x4 齐次变换矩阵 T_ab
 * @param Ad 输出 6x6 矩阵 (行优先)
 */
inline auto adjoint_matrix(const double* RTB_RESTRICT T, double* RTB_RESTRICT Ad) -> void {
    double R[9], p[3];
    decompose_transform(T, R, p);  // 假设你已有此函数

    // 计算 p_skew * R
    // p_skew = [0 -z  y; z 0 -x; -y x 0]
    // pR = p_skew * R
    double pR[9];
    pR[0] = -p[2] * R[3] + p[1] * R[6];
    pR[1] = -p[2] * R[4] + p[1] * R[7];
    pR[2] = -p[2] * R[5] + p[1] * R[8];

    pR[3] = p[2] * R[0] - p[0] * R[6];
    pR[4] = p[2] * R[1] - p[0] * R[7];
    pR[5] = p[2] * R[2] - p[0] * R[8];

    pR[6] = -p[1] * R[0] + p[0] * R[3];
    pR[7] = -p[1] * R[1] + p[0] * R[4];
    pR[8] = -p[1] * R[2] + p[0] * R[5];

    // 填充 6x6 矩阵
    mat_zero(Ad, 36);

    // Top-Left: R
    // Bottom-Right: R
    // Top-Right: p_skew * R
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            Ad[r * 6 + c] = R[r * 3 + c];              // TL
            Ad[(r + 3) * 6 + (c + 3)] = R[r * 3 + c];  // BR
            Ad[r * 6 + (c + 3)] = pR[r * 3 + c];       // TR
        }
    }
}

/**
 * @brief 运动旋量变换 (Twist Transformation)
 * @details V_A = Ad_T * V_B
 * 不需要显式构建 6x6 矩阵，直接计算以提高效率
 * @param T_ab 从 A 到 B 的变换矩阵
 * @param twist_b 在 B 系下的旋量 [v; w]
 * @param twist_a 输出：在 A 系下的旋量 [v; w]
 */
inline auto transform_twist_optimal(const double* T_ab, const double* twist_b, double* twist_a) -> void {
    double R[9], p[3];
    decompose_transform(T_ab, R, p);

    const double* v_b = twist_b;      // linear
    const double* w_b = twist_b + 3;  // angular

    // w_a = R * w_b
    double w_a[3];
    mat3_vec_mul(R, w_b, w_a);

    // v_a = R * v_b + p x (R * w_b) = R * v_b + p x w_a
    double v_rot[3];
    mat3_vec_mul(R, v_b, v_rot);

    double p_cross_wa[3];
    vec3_cross(p, w_a, p_cross_wa);

    double v_a[3];
    vec3_add(v_rot, p_cross_wa, v_a);

    // 输出
    std::memcpy(twist_a, v_a, 3 * sizeof(double));
    std::memcpy(twist_a + 3, w_a, 3 * sizeof(double));
}

/**
 * @brief 力旋量变换 (Wrench Transformation)
 * @details F_A = Ad_T^(-T) * F_B
 * Ad_T^(-T) = [ R        0 ]
 * [ [p]R     R ]  (对于 [v; w] 顺序的 Adjoint 的逆转置)
 * 这实际上等价于:
 * f_a = R * f_b
 * m_a = [p]R * f_b + R * m_b = p x (R * f_b) + R * m_b
 * @param T_ab 从 A 到 B 的变换矩阵
 * @param wrench_b 在 B 系下的力旋量 [f; m]
 * @param wrench_a 输出：在 A 系下的力旋量 [f; m]
 */
inline auto transform_wrench_optimal(const double* T_ab, const double* wrench_b, double* wrench_a) -> void {
    double R[9], p[3];
    decompose_transform(T_ab, R, p);

    const double* f_b = wrench_b;      // force
    const double* m_b = wrench_b + 3;  // moment

    // f_a = R * f_b
    double f_a[3];
    mat3_vec_mul(R, f_b, f_a);

    // m_a = R * m_b + p x f_a
    double m_rot[3];
    mat3_vec_mul(R, m_b, m_rot);

    double p_cross_fa[3];
    vec3_cross(p, f_a, p_cross_fa);

    double m_a[3];
    vec3_add(m_rot, p_cross_fa, m_a);

    // 输出
    std::memcpy(wrench_a, f_a, 3 * sizeof(double));
    std::memcpy(wrench_a + 3, m_a, 3 * sizeof(double));
}

// ============================================================================
// 几何反解 (Geometric Decomposition)
// ============================================================================

/**
 * @brief 螺旋参数结构体
 */
struct ScrewAxis {
    double axis[3];    // 归一化轴线方向 n
    double point[3];   // 轴线上距离原点最近的点 q
    double h;          // 节距 pitch
    double magnitude;  // 幅值 M (theta_dot 或 force_magnitude)
};

/**
 * @brief 从旋量反求几何参数
 * @details 对应数学推导:
 * M = |w|
 * n = w / M
 * h = w . v / M^2
 * q = (w x v) / M^2
 * 如果 w=0 (纯平移/纯力偶)，则:
 * M = |v|, n = v / M, h = inf
 * @param screw 输入旋量 [v; w] 或 [f; m]
 * @param out 输出结构体
 */
inline auto screw_decomposition(const double* screw_vec, ScrewAxis& out) -> void {
    const double* v = screw_vec;      // linear component
    const double* w = screw_vec + 3;  // angular component

    double w_norm = vec3_norm(w);

    if (w_norm > EPSILON) {
        // 一般情况 (General Case)
        out.magnitude = w_norm;

        // n = w / |w|
        out.axis[0] = w[0] / w_norm;
        out.axis[1] = w[1] / w_norm;
        out.axis[2] = w[2] / w_norm;

        double w_sq = w_norm * w_norm;

        // h = (w . v) / |w|^2
        out.h = vec3_dot(w, v) / w_sq;

        // q = (w x v) / |w|^2
        double w_cross_v[3];
        vec3_cross(w, v, w_cross_v);
        out.point[0] = w_cross_v[0] / w_sq;
        out.point[1] = w_cross_v[1] / w_sq;
        out.point[2] = w_cross_v[2] / w_sq;
    } else {
        // 纯平移/纯力偶 (Pure Translation / Pure Couple)
        // 此时 w = 0, 信息完全在 v 中
        double v_norm = vec3_norm(v);
        out.magnitude = v_norm;

        // 节距无穷大，我们在代码中通常设为0或者特殊标记，
        // 但在几何上，轴线方向由 v 决定
        if (v_norm > EPSILON) {
            out.axis[0] = v[0] / v_norm;
            out.axis[1] = v[1] / v_norm;
            out.axis[2] = v[2] / v_norm;
        } else {
            // 零向量
            vec_zero(out.axis, 3);
        }

        out.h = INFINITY;        // 表示纯平移
        vec_zero(out.point, 3);  // 轴线可以通过原点（不唯一）
    }
}

/**
 * @brief 从几何参数构建旋量 [v; w]
 * @details v = h*w - w x q
 * @param n 轴线方向 (单位向量)
 * @param q 轴线上一点
 * @param h 节距
 * @param magnitude 幅值
 * @param screw_out 输出 [v; w]
 */
inline auto screw_from_geometry(const double* n, const double* q, double h, double magnitude, double* screw_out) -> void {
    // w = M * n
    double w[3];
    vec3_scale(n, magnitude, w);

    if (std::isinf(h)) {
        // 纯平移: w=0, v = M * n
        screw_out[0] = w[0];
        screw_out[1] = w[1];
        screw_out[2] = w[2];
        screw_out[3] = 0;
        screw_out[4] = 0;
        screw_out[5] = 0;
    } else {
        // 一般情况
        // v = h * w + q x w
        double q_cross_w[3];
        vec3_cross(q, w, q_cross_w);

        double hv_term[3];
        vec3_scale(w, h, hv_term);  // h * w 因为 h 是 scalar

        screw_out[0] = hv_term[0] + q_cross_w[0];
        screw_out[1] = hv_term[1] + q_cross_w[1];
        screw_out[2] = hv_term[2] + q_cross_w[2];

        screw_out[3] = w[0];
        screw_out[4] = w[1];
        screw_out[5] = w[2];
    }
}

// ============================================================================
// 李代数 se(3) 指数映射 (Exponential Map)
// ============================================================================

/**
 * @brief 螺旋运动指数映射 se(3) -> SE(3)
 * @details 计算 T = exp(S * theta)，其中 S 是归一化旋量
 * 输入 twist 并不是归一化的，而是 twist = S * theta
 * 所以函数计算 T = exp([twist])
 * @param twist 输入螺旋向量 [v; w]
 * @param T_out 输出 4x4 变换矩阵
 */
inline auto axis_angle_to_transform(const double* twist, double* T_out) -> void {
    const double* v = twist;
    const double* w = twist + 3;

    double theta = vec3_norm(w);
    double R[9];
    double p[3];

    if (theta < EPSILON) {
        // 近似纯平移
        // R = I, p = v
        mat3_identity(R);
        p[0] = v[0];
        p[1] = v[1];
        p[2] = v[2];
    } else {
        // 罗德里格斯公式求 R
        // w_n = w / theta
        double w_n[3] = {w[0] / theta, w[1] / theta, w[2] / theta};
        rodrigues(w_n, theta, R);

        // 计算 p = (I*theta + (1-cos)*[w_n] + (theta-sin)*[w_n]^2) * (v/theta)
        // 更常用的 G(theta) * v 形式:
        // p = (I - R) * (w x v) / |w|^2 + (w . v) / |w|^2 * w * theta (screw trajectory)
        // 或者直接使用 G matrix 公式:
        // G = I*theta + (1-cos(theta))*[w_n] + (theta - sin(theta))*[w_n]^2
        // p = G * (v / theta)

        // 这里使用几何分解法合成更加直观且数值稳定：
        // 1. 分解出 h, q
        ScrewAxis axis;
        screw_decomposition(twist, axis);  // axis.magnitude = theta

        // 2. 旋转部分 R 已计算

        // 3. 平移部分 p = h * theta * n + (I - R) * q
        //    其中 h * theta * n 就是沿轴平移

        // slide = h * theta * n
        double slide[3];
        vec3_scale(axis.axis, axis.h * theta, slide);

        // rot_disp = (I - R) * q = q - R*q
        double Rq[3];
        mat3_vec_mul(R, axis.point, Rq);
        double rot_disp[3];
        vec3_sub(axis.point, Rq, rot_disp);

        vec3_add(slide, rot_disp, p);
    }

    compose_transform(R, p, T_out);
}

}  // namespace math
}  // namespace rtb

#endif  // RTB_MATH_HPP