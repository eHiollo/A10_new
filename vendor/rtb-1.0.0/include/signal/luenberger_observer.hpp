#pragma once

#include <memory>

namespace rtb {
namespace signal {

/**
 * @brief 多轴龙伯格运动学观测器
 * @details 通过位置测量值，提取平滑的位置、速度和加速度。计算极简，无矩阵运算。
 */
class LuenbergerObserver {
public:
    LuenbergerObserver();
    ~LuenbergerObserver();

    LuenbergerObserver(const LuenbergerObserver&) = delete;
    LuenbergerObserver& operator=(const LuenbergerObserver&) = delete;
    LuenbergerObserver(LuenbergerObserver&&) noexcept;
    LuenbergerObserver& operator=(LuenbergerObserver&&) noexcept;

    auto setInputSize(int input_size) -> void ;
    auto setDt(double dt) -> void ;

    /**
     * @brief 设置观测器增益
     * @param l1 位置反馈增益 (通常较大，如 100~300)
     * @param l2 速度反馈增益 (如 1000~5000)
     * @param l3 加速度反馈增益 (如 10000~50000)
     */
    auto setGains(double l1, double l2, double l3) -> void ;

    auto init(const double* q_init) -> void ;

    /**
     * @brief 更新测量值并输出状态
     * @param q_measure 当前编码器测量角度
     * @param q_hat 输出估计角度
     * @param v_hat 输出估计角速度
     * @param a_hat 输出估计角加速度
     */
    auto update(const double* q_measure, double* q_hat, double* v_hat, double* a_hat) -> void ;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace signal
}  // namespace rtb