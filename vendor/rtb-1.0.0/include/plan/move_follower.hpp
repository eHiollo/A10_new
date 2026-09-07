/**
 * @file move_follower.hpp
 * @brief 独立的轨迹跟随算法模块 (重构版 - 使用统一数学库)
 * @details 支持位置与姿态的平滑跟踪，自动满足速度和加速度约束
 */

#ifndef RTB_MOVE_FOLLOWER_HPP
#define RTB_MOVE_FOLLOWER_HPP

#include "core/macro.hpp"
#include <memory>

namespace rtb {
namespace plan {

class MoveFollower {
public:
    // 设置控制周期 (s), 默认 0.001
    auto setDt(double dt) -> void ;

    // 设置最大约束参数
    auto setMaxVel(double v) -> void ;
    auto setMaxAcc(double a) -> void ;
    auto setMaxAngVel(double w) -> void ;
    auto setMaxAngAcc(double alpha) -> void ;

    // 设置目标状态
    // pm: 4x4 齐次变换矩阵 (行优先，index 3,7,11 为位置)
    // va: 6维速度 [vx, vy, vz, wx, wy, wz]
    auto setTargetPm(const double* pm) -> void ;
    auto setTargetVa(const double* va) -> void ;

    // 设置当前跟随者状态 (用于初始化或重置)
    auto setFollowPm(const double* pm) -> void ;
    auto setFollowVa(const double* va) -> void ;

    // 核心计算函数：推演一个时间步长 dt
    // 输出更新后的 follow_pm (4x4) 和 follow_va (6x1)，以及期望加速度 follow_acc(6x1)
    auto moveDtAndGetResult(double* follow_pm, double* follow_va = nullptr, double* follow_acc = nullptr) -> void ;

    // 返回追踪的剩余时间
    auto estimateLeftT(double* p_time = nullptr, double* r_time = nullptr) -> double ;

    // 重置内部状态（用于重新执行指令时清空历史数据）
    auto reset() -> void ;

    MoveFollower();
    ~MoveFollower();
    HPP_DECLARE_BIG_FOUR(MoveFollower);

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace plan
}  // namespace rtb

#endif  // RTB_MOVE_FOLLOWER_HPP
