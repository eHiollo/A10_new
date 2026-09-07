#ifndef RTB_PLAN_MPC_SMOOTHER_H_
#define RTB_PLAN_MPC_SMOOTHER_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <memory>

namespace rtb {
namespace plan {
namespace dev {

/// @brief 基于 OSQP 的多关节在线轨迹平滑器 (支持 P/V/A 硬约束与 Jerk 软约束)
class MpcSmoother {
public:
    /// @brief 构造函数
    /// @param num_joints 关节数量 (例如 7 轴机械臂传入 7)
    /// @param horizon 预测步长 H (决定前瞻视野，通常 20~50)
    /// @param dt 控制周期 (秒)
    MpcSmoother(int num_joints, int horizon, double dt);
    ~MpcSmoother();

    // 禁用拷贝与赋值
    MpcSmoother(const MpcSmoother&) = delete;
    MpcSmoother& operator=(const MpcSmoother&) = delete;

    /// @brief 初始化平滑器内部状态 (在启动或发生突变重置时调用)
    /// @param init_p 初始关节位置 [num_joints]
    /// @param init_v 初始关节速度 [num_joints]
    /// @param init_a 初始关节加速度 [num_joints]
    auto init(const Eigen::VectorXd& init_p, const Eigen::VectorXd& init_v, const Eigen::VectorXd& init_a) -> bool;

    // ================== 约束设置 (硬约束) ==================
    auto setMaxPos(const Eigen::VectorXd& max_p) -> void;
    auto setMinPos(const Eigen::VectorXd& min_p) -> void;
    auto setMaxVel(const Eigen::VectorXd& max_v) -> void;
    auto setMinVel(const Eigen::VectorXd& min_v) -> void;
    auto setMaxAcc(const Eigen::VectorXd& max_a) -> void;
    auto setMinAcc(const Eigen::VectorXd& min_a) -> void;

    // ================== 约束设置 (软约束/目标域) ==================
    /// @brief 设置理想的最大/最小加加速度
    auto setMaxJerk(const Eigen::VectorXd& max_j) -> void;
    auto setMinJerk(const Eigen::VectorXd& min_j) -> void;

    // ================== 权重设置 ==================
    /// @brief 设置目标函数权重
    /// @param w_p 位置追踪权重 (Q) - 越大追踪越紧
    /// @param w_j Jerk惩罚权重 (R) - 越大越平滑，越小越趋近时间最优
    /// @param w_slack 松弛变量惩罚权重 (W) - 必须非常大 (例如 1e8)，非绝境不触发
    auto setWeights(double w_p, double w_j, double w_slack) -> void;

    // ================== 核心运行接口 ==================
    /// @brief 执行单步 MPC 优化，获取下一拍的安全控制指令
    /// @param current_p 当前时刻物理真实位置 [num_joints]
    /// @param current_v 当前时刻物理真实速度 [num_joints]
    /// @param current_a 当前时刻物理真实加速度 [num_joints]
    /// @param ref_p 未来 H 步的参考目标位置 (长度为 num_joints * horizon 的一维展开向量)
    /// @param out_next_p [输出] 安全平滑后的下一拍位置指令
    /// @param out_next_v [输出] 安全平滑后的下一拍速度指令
    /// @param out_next_a [输出] 安全平滑后的下一拍加速度指令
    /// @return 求解是否成功 (如果 P/V/A 被逼入死角且松弛变量也无法拯救，会返回 false)
    auto step(const Eigen::VectorXd& current_p, const Eigen::VectorXd& current_v, const Eigen::VectorXd& current_a,
        const Eigen::VectorXd& ref_p, Eigen::VectorXd& out_next_p, Eigen::VectorXd& out_next_v,
        Eigen::VectorXd& out_next_a) -> bool;

private:
    // PIMPL 惯用手法，隐藏复杂的 Eigen 稀疏矩阵拼接逻辑与 OSQP 实例
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace dev
}  // namespace plan
}  // namespace rtb

#endif  // RTB_PLAN_MPC_SMOOTHER_H_