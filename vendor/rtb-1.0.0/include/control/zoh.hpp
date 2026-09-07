/**
 * @file zoh.hpp
 * @brief 通用零阶保持器（ZOH）离散化 — s 域传递函数 → z 域传递函数
 * @details
 * 实现三类核心算法：
 *
 * 1. 传递函数 → 连续状态空间（可控标准型）
 *    G(s) = [b_{n-1}s^{n-1} + … + b_0] / [d_n s^n + d_{n-1}s^{n-1} + … + d_0]
 *    init() 内部自动用 d_n 对所有系数归一化，用户无需手动处理
 *    → (A, B, C) 可控标准型
 *
 * 2. ZOH 离散化（分块矩阵指数法）
 *    M = [A  B; 0  0] · Ts
 *    exp(M) = [Ad  Bd; 0  I]
 *
 * 3. 离散状态空间 → 传递函数（Faddeev-Leverrier）
 *    G(z) = Cd · (zI - Ad)^(-1) · Bd
 *    输出分子/分母系数多项式
 */
#ifndef RTB_CONTROL_ZOH_HPP
#define RTB_CONTROL_ZOH_HPP

#include <memory>

namespace rtb {
namespace control {

class ZOH {
public:
    // ---- 构造与析构 ----
    ZOH();
    ~ZOH();

    // ---- Rule of Three/Five（深拷贝语义） ----
    ZOH(const ZOH& other);
    auto operator=(const ZOH& other) -> ZOH&;
    ZOH(ZOH&& other) noexcept;
    auto operator=(ZOH&& other) noexcept -> ZOH&;

    /** @brief 采样周期，单位秒，必须 > 0 */
    auto ts() const -> double;
    auto setTs(double ts) -> void;

    /** @brief 连续系统的阶次，在 init() 后有效 */
    auto order() const -> int;

    /** @brief 是否已完成离散化 */
    auto isReady() const -> bool;

    /**
     * @brief 初始化连续系统参数，要求严格真分式（分母阶次 > 分子阶次）
     * @param num 分子系数，b_{n-1}, …, b_0（长度 num_len）
     * @param num_len 分子系数个数（= 分母阶次）
     * @param den 分母系数，d_{n}, d_{n-1}, …, d_0（长度 den_len = 分母阶次 + 1）
     * @param den_len 分母系数个数（含首项 d_n）
     * @param ts   采样周期 Ts，必须 > 0
     * @return 参数合法返回 true，否则 false
     * @note 用户传入完整分母系数（含首项 d_n），init() 内部自动用 den[0] 对所有系数归一化
     */
    auto init(const double* num, int num_len, const double* den, int den_len, double ts) -> bool;

    /**
     * @brief 执行 ZOH 离散化
     * @return 成功 true，矩阵指数计算失败时返回 false
     * @details 先调用 init() 再调用本方法。内部执行：
     *          1. 可控标准型构造 (A, B, C)
     *          2. 分块矩阵指数计算 (Ad, Bd)
     *          3. Faddeev-Leverrier 恢复传递函数
     */
    auto discretize() -> bool;

    // ---- 获取离散化结果 ----

    /** @brief 离散系统分子系数，长度为 order()（对应 b_{n-1}, …, b_0） */
    auto numZ() const -> const double*;
    /** @brief 离散系统分母系数，长度为 order() + 1（对应 1, a_{n-1}, …, a_0） */
    auto denZ() const -> const double*;

    /** @brief 离散状态矩阵 Ad（n × n，行优先） */
    auto ad() const -> const double*;
    /** @brief 离散输入矩阵 Bd（n × 1） */
    auto bd() const -> const double*;
    /** @brief 离散输出矩阵 Cd（1 × n），等于连续系统的 C */
    auto cd() const -> const double*;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace control
}  // namespace rtb

#endif  // RTB_CONTROL_ZOH_HPP