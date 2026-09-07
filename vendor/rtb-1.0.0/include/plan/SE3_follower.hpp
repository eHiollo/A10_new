/**
 * @file SE3_follower.hpp
 * @brief 基于 SE(3) 流形的时间最优同步追踪器
 * @details
 * 1. 有状态设计: 内部存储当前位姿和速度，类似 MoveFollower
 * 2. 严格同步: 采用梯形速度规划实现位姿耦合追踪
 * 3. 局部时间最优: 精确的可行性检查和追赶策略
 * 4. 鲁棒性: 线性化死区 + 原理性的收敛判断
 *
 * 接口完全兼容 MoveFollower，可直接替换使用
 *
 * --------------------------------------------------------------------------
 * 六维速度 va = [vx,vy,vz,wx,wy,wz] 的语义
 * （输入 setTargetVa/setFollowVa，输出 moveDt 的 follow_va）
 * --------------------------------------------------------------------------
 * 【明确结论】
 *   在本实现中，该六维量不是现代机器人学旋量理论里严格定义的
 *   空间旋量 (spatial twist，常记 V_s)，也不是物体旋量 (body twist，常记 V_b)。
 *   它是工程中最常用的「混合导数 / 笛卡尔速度」(mixed derivative)：
 *
 *     dot{X} 约定为  [  tcp 在基坐标系下的线速度 ;  工具系相对基系的角速度在基系下的表达  ]
 *
 *   即前 3 维与后 3 维均在基坐标系 (base frame) 下分量表示，与 setMaxVel/setMaxAcc
 *   及 setMaxAngVel/setMaxAngAcc 的物理约束一一对应。
 *
 * 【平移 vx, vy, vz】
 *   TCP（工具尖端）在基坐标系下的线速度 [m/s]。沿基座 X/Y/Z 轴走直线时，只需令对应分量为期望速率；
 *   该表达不随当前末端姿态改变「基系下分量」的定义方式，便于与线速度/线加速度限幅对接。
 *
 * 【旋转 wx, wy, wz】
 *   工具坐标系相对于基坐标系的角速度，在基系下的 3 维向量 [rad/s]：方向为瞬时转轴（在基系中），
 *   模长为角速率；与角速度/角加速度限幅一致。
 *
 * 【与几何雅可比的对接】
 *   常见几何雅可比 J_geo 将关节速度映射为 [ p_dot ; omega ]，且 p_dot、omega 往往与上述混合笛卡尔速度
 *   同语义。因此可将本追踪器输出的 follow_va 直接用于速度级逆解（如 dot_q = inv(J) * v_cmd），
 *   一般无需再做 v - omega x p 等空间/物体旋量之间的显式转换（除非你所用的 J 定义与本约定不同，
 *   那时需先统一雅可比与速度的坐标约定）。
 * --------------------------------------------------------------------------
 */

#ifndef RTB_SE3_FOLLOWER_HPP
#define RTB_SE3_FOLLOWER_HPP

#include "core/macro.hpp"
#include <memory>

namespace rtb {
namespace plan {

class SE3Follower {
public:
    // ========================================================================
    // 配置接口
    // ========================================================================
    auto setDt(double dt) -> void ;
    auto setMaxVel(double v) -> void ;
    auto setMaxAcc(double a) -> void ;
    auto setMaxAngVel(double w) -> void ;
    auto setMaxAngAcc(double alpha) -> void ;
    auto setMaxJerk(double j) -> void ;
    auto setMaxAngJerk(double j_ang) -> void ;

    // ========================================================================
    // 状态设置接口
    // ========================================================================
    auto setTargetPm(const double* pm) -> void ;
    /// 目标六维速度，语义见文件头 @details（基系下线速度 + 基系下角速度，非严格 spatial/body twist）
    auto setTargetVa(const double* va) -> void ;
    auto setFollowPm(const double* pm) -> void ;
    /// 跟随六维速度，约定与 setTargetVa 相同
    auto setFollowVa(const double* va) -> void ;

    // ========================================================================
    // 核心更新函数
    // ========================================================================
    /**
     * @brief 执行一个控制周期的更新
     * @param follow_pm  [输出] 更新后的跟随位姿 (4x4 矩阵)
     * @param follow_va  [输出] 更新后的跟随速度 (6 维)，语义同文件头：基系 TCP 线速度 + 基系下角速度
     * @param follow_acc [输出] 计算出的加速度 (6维)
     */
    auto moveDtAndGetResult(double* follow_pm, double* follow_va = nullptr, double* follow_acc = nullptr) -> void ;

    /**
     * @brief 返回追踪的剩余时间估计
     * @param p_time [输出] 平移剩余时间估计
     * @param r_time [输出] 旋转剩余时间估计
     * @return 剩余时间估计
     */
    auto estimateLeftT(double* p_time = nullptr, double* r_time = nullptr) -> double ;

    /**
     * @brief 执行带有 Jerk 平滑限制的周期更新 (S 型曲线生成)
     * @param smooth_pm       [输出] 平滑后的跟随位姿
     * @param smooth_va       [输出] 平滑后的跟随速度
     * @param smooth_acc      [输出] 平滑后的跟随加速度
     * @param smooth_jerk_out [输出] (可选) 真实的物理 Jerk (加加速度)
     * @param inner_cmd_acc   [输出] (可选) 底层原始的方波加速度指令
     */
    auto moveSmoothDtAndGetResult(double* smooth_pm, double* smooth_va = nullptr, double* smooth_acc = nullptr,
        double* smooth_jerk = nullptr, double* inner_cmd_acc = nullptr) -> void ;
    /**
     * @brief 重置到初始状态
     */
    auto reset() -> void ;

    // 构造/析构函数和 Big Four
    SE3Follower();
    ~SE3Follower();
    HPP_DECLARE_BIG_FOUR_NOEXCEPT(SE3Follower);

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;

    // 内部辅助函数
    auto check_lin_vel_feasibility(const double* v0, const double* v1, const double* v_avg) -> bool ;
    auto check_ang_vel_feasibility(const double* w0, const double* w1, const double* w_avg) -> bool ;
};

}  // namespace plan
}  // namespace rtb

#endif  // RTB_SE3_FOLLOWER_HPP
