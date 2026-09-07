/**
 * @file pid.hpp
 * @brief 通用 PID 控制器 — 支持抗积分饱和（Integral Anti-Windup）、输出限幅与微分滤波的工业级 PID
 */

#ifndef RTB_CONTROL_PID_HPP
#define RTB_CONTROL_PID_HPP

#include <memory>


namespace rtb {
namespace control {

/**
 * @brief 通用 PID 控制器
 *
 * 支持两种模式：
 *   - 位置式 PID：标准并行形式，输出 = P + I + D
 *   - 增量式 PID：输出增量，适合步进电机等需要增量指令的场景
 *
 * 内置功能：
 *   - 积分抗饱和（Anti-Windup）：当输出触及饱和上限/下限时自动冻结积分累积
 *   - 输出限幅：将最终输出钳制在 [outMin, outMax] 范围内
 *   - 微分滤波（可选）：对微分项施加一阶低通滤波，抑制高频噪声
 */
class PID {
public:
    // ---- 构造与析构 ----
    PID();
    ~PID();

    // ---- Rule of Three/Five（深拷贝语义） ----
    PID(const PID& other);
    auto operator=(const PID& other) -> PID&;
    PID(PID&& other) noexcept;
    auto operator=(PID&& other) noexcept -> PID&;

    // ---- 参数设置（Getter / Setter） ----
    /** @brief 比例增益 (Kp) */
    auto kp() const -> double;
    auto setKp(double v) -> void;

    /** @brief 积分增益 (Ki) */
    auto ki() const -> double;
    auto setKi(double v) -> void;

    /** @brief 微分增益 (Kd) */
    auto kd() const -> double;
    auto setKd(double v) -> void;

    /** @brief 设定点 */
    auto setpoint() const -> double;
    auto setSetPoint(double v) -> void;

    /** @brief 输出最小值 */
    auto outputMin() const -> double;
    /** @brief 输出最大值 */
    auto outputMax() const -> double;
    /** @brief 设置 PID 输出饱和上限（绝对值） */
    auto setOutputLimit(double minVal, double maxVal) -> void;

    /** @brief 积分最小值 */
    auto integralMin() const -> double;
    /** @brief 积分最大值 */
    auto integralMax() const -> double;
    /** @brief 设置积分饱和上限（绝对值），通常等于或略小于输出饱和限 */
    auto setIntegralLimit(double minVal, double maxVal) -> void;

    /** @brief 微分滤波时间常数 */
    auto derivativeFilterTau() const -> double;
    /** @brief 设置微分低通滤波时间常数（tau≈0 时等同于无滤波） */
    auto setDerivativeFilterTau(double tau) -> void;

    /** @brief 控制周期 dt (s) */
    auto dt() const -> double;
    /** @brief 设置控制周期 dt (s) */
    auto setDt(double dt) -> void;

    // ---- 在线调参与查询 ----
    /** @brief 在线调参（一次设置 Kp/Ki/Kd），不影响当前积分 */
    auto tune(double Kp, double Ki, double Kd) -> void;

    /** @brief 查询当前积分值 */
    auto integral() const -> double;

    // ---- 核心算法接口 ----

    /**
     * @brief 核心更新函数
     * @param measurement 当前反馈测量值
     * @return 经过饱和限幅后的 PID 输出
     *
     * 内部使用 e = setpoint - measurement 作为误差。
     * 调用此函数前需先设置 setpoint、dt。
     */
    auto update(double measurement) -> double;

    /**
     * @brief 更新 + 前馈
     * @param measurement 当前反馈测量值
     * @param feedforward 前馈项（原始值，不经过 PID 路径）
     * @return 饱和限幅后的 (PID输出 + feedforward)
     */
    auto update(double measurement, double feedforward) -> double;

    // ---- 分量查询 ----
    /** @brief 最近一次 update() 的比例项输出 */
    auto pTerm() const -> double;
    /** @brief 最近一次 update() 的积分项输出 */
    auto iTerm() const -> double;
    /** @brief 最近一次 update() 的微分项输出 */
    auto dTerm() const -> double;

    // ---- 状态复位 ----

    /** @brief 复位所有内部状态（积分、历史误差等） */
    auto reset() -> void;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace control
}  // namespace rtb

#endif  // RTB_CONTROL_PID_HPP