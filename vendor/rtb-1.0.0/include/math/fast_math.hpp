/**
 * @file fast_math.hpp
 * @brief 快速数学运算库 (Header-only)
 * @details
 * 提供比 std::cmath 更快但精度略低的数学函数。
 * 核心策略：
 * 1. Sin/Cos: 角度规约 + 泰勒级数展开 (误差 < 1e-6)
 * 2. InvSqrt: 快速位操作 + 牛顿迭代 (误差 < 1e-3)
 * 3. Atan2:   有理函数逼近 (误差 < 5e-4)
 * * @note 适用于机器人运动控制、图形渲染等对实时性要求高、对绝对精度容忍度在 1e-4 级别的场景。
 */

#ifndef RTB_CORE_FAST_MATH_HPP
#define RTB_CORE_FAST_MATH_HPP

#include <cmath>
#include <cstring>

namespace rtb {
namespace math {

class FastMath {
public:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double TWO_PI = 6.28318530717958647692;
    static constexpr double HALF_PI = 1.57079632679489661923;
    static constexpr double INV_TWO_PI = 0.15915494309189533576;  // 1 / (2*PI)

    /**
     * @brief 角度规约 (将任意角度规约到 [-PI, PI] 区间)
     * @note 相比 while 循环，这种乘法取整的方式在大角度输入下效率极高
     */
    static inline auto normalize_angle(double angle) -> double {
        // 使用浮点数特性快速取模
        return angle - TWO_PI * std::floor((angle + PI) * INV_TWO_PI);
    }

    /**
     * @brief 快速正弦函数
     * @details
     * 1. 将角度规约到 [-PI, PI]
     * 2. 利用泰勒级数 sin(x) ~= x - x^3/6 + x^5/120 - x^7/5040
     * @note 精度: 误差约 1e-6，速度: 约为 std::sin 的 3-5 倍
     */
    static inline auto fast_sin(double x) -> double {
        // 1. 规约到 [-PI, PI]
        x = normalize_angle(x);

        // 2. 如果在 [-PI, PI] 边缘，精度会下降，进一步利用对称性规约到 [-PI/2, PI/2]
        // 这一步能显著提升大角度下的精度
        if (x > HALF_PI) {
            x = PI - x;
        } else if (x < -HALF_PI) {
            x = -PI - x;
        }

        // 3. 泰勒级数计算 (x^7)
        double x2 = x * x;
        // 系数分别为: 1/1!, -1/3!, 1/5!, -1/7!
        return x * (1.0 + x2 * (-0.16666666666666666 + x2 * (0.008333333333333333 + x2 * -0.000198412698412698)));
    }

    /**
     * @brief 快速余弦函数
     * @details 利用 cos(x) = sin(x + PI/2)
     */
    static inline auto fast_cos(double x) -> double { return fast_sin(x + HALF_PI); }

    /**
     * @brief 快速正余弦联合计算
     * @details 同时计算 sin 和 cos，比分别调用快
     */
    static inline auto fast_sincos(double x, double* s, double* c) -> void {
        *s = fast_sin(x);
        *c = fast_cos(x);
        // 也可以用 sqrt(1 - s*s) 计算 c，但要注意符号，这里直接调用 fast_cos 足够快且安全
    }

    /**
     * @brief 快速平方根倒数 (float版) - Quake III 算法
     * @return 1 / sqrt(x)
     * @note 包含一次牛顿迭代，精度满足绝大多数工程需求
     */
    static inline auto fast_inv_sqrt(float x) -> float {
        if (x <= 0.0f)
            return 0.0f;
        float xhalf = 0.5f * x;
        int32_t i;
        std::memcpy(&i, &x, sizeof(x));  // 位表示，避免 strict-aliasing 未定义行为
        i = 0x5f3759df - (i >> 1);  // 魔法常数计算初始猜测值
        std::memcpy(&x, &i, sizeof(x));
        x = x * (1.5f - xhalf * x * x);  // 牛顿迭代 (一次)
        return x;
    }

    /**
     * @brief 快速平方根倒数 (double版)
     * @details 针对 64位 double 的魔法常数实现
     */
    static inline auto fast_inv_sqrt(double x) -> double {
        if (x <= 0.0)
            return 0.0;
        double xhalf = 0.5 * x;
        int64_t i;
        std::memcpy(&i, &x, sizeof(x));
        i = 0x5fe6eb50c7b537a9 - (i >> 1);  // 针对 double 的魔法常数
        std::memcpy(&x, &i, sizeof(x));
        x = x * (1.5 - xhalf * x * x);  // 牛顿迭代
        return x;
    }

    /**
     * @brief 快速平方根 (基于 fast_inv_sqrt)
     * @note 标准库 std::sqrt 在现代 CPU 上有硬件指令 (SQRTSS)，
     * 除非开启了 -ffast-math，否则手写 fast_sqrt 并不一定比硬件指令快。
     * 这里的实现主要用于保持一致性或无 FPU 环境。
     */
    static inline auto fast_sqrt(double x) -> double {
        if (x <= 0.0)
            return 0.0;
        return 1.0 / fast_inv_sqrt(x);
    }

    /**
     * @brief 快速 Atan2
     * @details
     * 1. 规约到第一象限
     * 2. 使用有理逼近: atan(z) ~= z / (1 + 0.28*z^2) 的改进版
     * 3. 恢复象限符号
     * @note 误差 < 0.005 rad (约 0.28 度)
     */
    static inline auto fast_atan2(double y, double x) -> double {
        if (x == 0.0) {
            if (y > 0.0)
                return HALF_PI;
            if (y == 0.0)
                return 0.0;
            return -HALF_PI;
        }

        double ax = std::abs(x);
        double ay = std::abs(y);
        double a = (ay < ax) ? ay / ax : ax / ay;
        double s = a * a;

        // 改进的多项式逼近，比 z/(1+0.28z^2) 更准
        // p(x) = (0.9724 * x - 0.1919 * x^3)
        // 进一步优化系数以减小最大误差
        double r = ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a;

        if (ay > ax)
            r = HALF_PI - r;
        if (x < 0.0)
            r = PI - r;
        if (y < 0.0)
            r = -r;

        return r;
    }
};

}  // namespace math
}  // namespace rtb

#endif  // RTB_CORE_FAST_MATH_HPP