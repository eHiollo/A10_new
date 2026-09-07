/**
 * @file discrete_plant.hpp
 * @brief 离散差分方程求解器 — 基于环形队列的直接型 I 结构（Direct Form I）
 */

#ifndef RTB_CONTROL_DISCRETE_PLANT_HPP
#define RTB_CONTROL_DISCRETE_PLANT_HPP

#include <memory>

namespace rtb {
namespace control {

/**
 * @brief 离散传递函数差分方程求解器
 *
 * 实现基于环形队列的直接型 I 结构（Direct Form I），无数据搬运开销。
 * 对应传递函数：
 *   G(z) = [b_1·z⁻¹ + … + b_n·z⁻ⁿ] / [1 + a_1·z⁻¹ + … + a_n·z⁻ⁿ]
 * 差分方程：
 *   y[k] = Σ b_i·u[k-i] - Σ a_i·y[k-i]
 */
class DiscretePlant {
public:
    // ---- 构造与析构 ----
    DiscretePlant();
    ~DiscretePlant();

    // ---- Rule of Three/Five（深拷贝语义） ----
    DiscretePlant(const DiscretePlant& other);
    auto operator=(const DiscretePlant& other) -> DiscretePlant&;
    DiscretePlant(DiscretePlant&& other) noexcept;
    auto operator=(DiscretePlant&& other) noexcept -> DiscretePlant&;

    /**
     * @brief 初始化系统系数
     * @param num 分子系数 b₁,…,b_n，长度 n（不含 b₀，因严格真分式 b₀ ≡ 0）
     * @param den 分母系数 a₁,…,a_n，长度 n（不含首项 1.0）
     * @param n   系统阶次
     * @return 参数合法返回 true，否则 false
     */
    auto init(const double* num, const double* den, int n) -> bool;

    /** @brief 系统阶次 */
    auto order() const -> int;

    /**
     * @brief 递推一步差分方程
     * @param current_u 当前输入 u[k]
     * @return 当前输出 y[k]
     */
    auto step(double current_u) -> double;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace control
}  // namespace rtb

#endif  // RTB_CONTROL_DISCRETE_PLANT_HPP
