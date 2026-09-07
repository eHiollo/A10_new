/**
 * @file single_axis_follower.hpp
 * @brief 单维度追踪器 - 满足速度和加速度约束的平滑跟随
 * @details 用于单个关节、单个坐标轴的平滑跟踪，保证实时性
 */

#ifndef RTB_SINGLE_AXIS_FOLLOWER_HPP
#define RTB_SINGLE_AXIS_FOLLOWER_HPP

namespace rtb {
namespace plan {

/**
 * @class SingleAxisFollower
 * @brief 单维度追踪器
 *
 * 功能：
 * - 输入：目标位置、目标速度
 * - 输出：平滑的跟随位置、速度、加速度
 * - 约束：满足最大速度和最大加速度限制
 */
class SingleAxisFollower {
public:
    SingleAxisFollower();
    ~SingleAxisFollower() = default;

    /**
     * @brief 设置控制周期
     * @param dt 周期时间（秒），默认 0.001
     */
    auto setDt(double dt) -> void ;

    /**
     * @brief 设置最大速度
     * @param v_max 最大速度
     */
    auto setMaxVel(double v_max) -> void ;

    /**
     * @brief 设置最大加速度
     * @param a_max 最大加速度
     */
    auto setMaxAcc(double a_max) -> void ;

    /**
     * @brief 设置目标状态
     * @param target_pos 目标位置
     * @param target_vel 目标速度（可选，默认0）
     */
    auto setTarget(double target_pos, double target_vel = 0.0) -> void ;

    /**
     * @brief 设置当前跟随状态（用于初始化）
     * @param follow_pos 跟随位置
     * @param follow_vel 跟随速度（可选，默认0）
     */
    auto setFollow(double follow_pos, double follow_vel = 0.0) -> void ;

    /**
     * @brief 执行一个时间步长的跟踪计算
     * @param out_pos 输出：跟随位置
     * @param out_vel 输出：跟随速度
     * @param out_acc 输出：期望加速度
     */
    auto moveDtAndGetResult(double& out_pos, double& out_vel, double& out_acc) -> void ;

    /**
     * @brief 评估剩余追踪时间
     */
    auto estimateLeftT() -> double ;

    /**
     * @brief 重置内部状态
     */
    auto reset() -> void ;

    /**
     * @brief 获取当前跟随位置
     */
    auto getFollowPos() const -> double { return follow_pos_; }

    /**
     * @brief 获取当前跟随速度
     */
    auto getFollowVel() const -> double { return follow_vel_; }

private:
    double dt_{0.001};    // 控制周期
    double v_max_{1.0};   // 最大速度
    double a_max_{10.0};  // 最大加速度

    double target_pos_{0.0};  // 目标位置
    double target_vel_{0.0};  // 目标速度
    double follow_pos_{0.0};  // 跟随位置
    double follow_vel_{0.0};  // 跟随速度
};

}  // namespace plan
}  // namespace rtb

#endif  // RTB_SINGLE_AXIS_FOLLOWER_HPP
