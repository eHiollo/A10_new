#pragma once

#include <memory>
#include <vector>

namespace rtb {
namespace plan {

/**
 * @brief 多轴五次多项式混合过渡规划器
 * @details 采用 PIMPL
 * 惯用语法隐藏内部实现。用于在线重规划时，桥接非零初始状态与新的规划目标，保证位置、速度、加速度连续 (C2 连续)。
 */
class QuinticBlender {
public:
    QuinticBlender();
    ~QuinticBlender();

    // 禁用拷贝，允许移动
    QuinticBlender(const QuinticBlender&) = delete;
    QuinticBlender& operator=(const QuinticBlender&) = delete;
    QuinticBlender(QuinticBlender&&) noexcept;
    QuinticBlender& operator=(QuinticBlender&&) noexcept;

    /**
     * @brief 初始化规划器维度
     * @param input_size 关节数量
     */
    auto setInputSize(int input_size) -> void ;

    /**
     * @brief 设置物理约束（用于评估生成的曲线是否可行）
     */
    auto setMaxVel(const std::vector<double>& max_vel) -> void ;
    auto setMaxAcc(const std::vector<double>& max_acc) -> void ;

    /**
     * @brief 生成五次多项式系数
     * @param q0, v0, a0 起点位置、速度、加速度数组 (大小为 input_size)
     * @param q1, v1, a1 终点位置、速度、加速度数组 (大小为 input_size)
     * @param Tb 过渡时间
     */
    auto generate(const double* q0, const double* v0, const double* a0, const double* q1, const double* v1,
        const double* a1, double Tb) -> void ;

    /**
     * @brief 获取时间 t 时刻的状态
     * @param t 当前时间 (0 <= t <= Tb)
     * @param q, v, a 输出的位置、速度、加速度 (传入 nullptr 则不计算该项)
     */
    auto evaluate(double t, double* q, double* v, double* a) const -> void ;

    /**
     * @brief 检查当前生成的五次多项式在 [0, Tb] 区间内是否满足最大速度和最大加速度约束
     * @param sample_dt 离散采样步长 (例如 0.001)
     * @return true 表示满足约束，可以安全执行；false 表示超限，需要增大 Tb 重新生成
     */
    auto checkLimits(double sample_dt) const -> bool ;

    /**
     * @brief 获取生成的过渡时间 Tb
     */
    auto getTb() const -> double ;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace plan
}  // namespace rtb