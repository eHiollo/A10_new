/**
 * @file srs_planner.hpp
 * @brief S-R-S七自由度冗余机械臂运动规划器 (重构版 - 使用统一数学库)
 * @details 支持逆运动学、臂角优化、可行域计算等功能
 */

#ifndef RTB_SRS_PLANNER_HPP
#define RTB_SRS_PLANNER_HPP

#include <array>
#include <string>

namespace rtb {
namespace plan {

// 数组长度约定：关节角 q 为 7 (JointVec)；位姿矩阵 T 为 16 行优先 (Mat4x4)；8 组解 solutions 为 56 = 8*7，布局
// solutions[i*7+j]
constexpr size_t RTB_JOINT_DOF = 7;
constexpr size_t RTB_MAT4_SIZE = 16;
/** 位姿 pq：位置 xyz + 四元数 [qw,qx,qy,qz]，与 math::pm2pq / pq2pm 一致 */
constexpr size_t RTB_PQ_SIZE = 7;
constexpr size_t RTB_IK8_SOLUTIONS_SIZE = 8 * RTB_JOINT_DOF;  // 56

using Interval = std::array<double, 2>;  // [min, max]

// 实时系统优化：固定大小的区间缓冲区（避免动态内存分配）
constexpr size_t MAX_INTERVALS = 100;  // 最大区间数（足够容纳所有情况）

struct IntervalArray {
    size_t count{0};
    std::array<Interval, MAX_INTERVALS> data;

    inline auto clear() noexcept -> void { count = 0; }

    inline auto push_back(const Interval& interval) noexcept -> void {
        if (count < MAX_INTERVALS) {
            data[count++] = interval;
        }
    }

    inline auto empty() const noexcept -> bool { return count == 0; }

    inline auto size() const noexcept -> size_t { return count; }

    inline Interval& operator[](size_t i) { return data[i]; }

    inline const Interval& operator[](size_t i) const { return data[i]; }

    // 迭代器支持（用于兼容算法）
    inline auto begin() -> Interval* { return data.data(); }

    inline auto end() -> Interval* { return data.data() + count; }

    inline auto begin() const -> const Interval* { return data.data(); }

    inline auto end() const -> const Interval* { return data.data() + count; }

    // 判断某个值是否落入任一区间（含端点）
    inline auto contains(double value, double tol = 1e-14) const noexcept -> bool {
        for (size_t i = 0; i < count; ++i) {
            if (value >= data[i][0] - tol && value <= data[i][1] + tol) {
                return true;
            }
        }
        return false;
    }
};

// 机器人参数结构体
struct RobotParams {
    // 几何参数 (米)
    double d_bs = 0.2665;  // 基座到肩部
    double d_se = 0.2555;  // 肩部到肘部
    double d_ew = 0.2555;  // 肘部到腕部
    double d_wt = 0.1665;  // 腕部到工具端（实际到法兰）

    // 关节位置限位 (弧度)，对称区间 [q_min, q_max]
    double q_min[7];
    double q_max[7];

    /** 角速度上下限 (rad/s)，默认 [-0.5, +0.5] */
    double q_vel_min[7];
    double q_vel_max[7];
    /** 角加速度 (rad/s²)，默认 [-5, +5] */
    double q_acc_min[7];
    double q_acc_max[7];
    /** 角加加速度 / jerk (rad/s³)，默认 [-50, +50] */
    double q_jerk_min[7];
    double q_jerk_max[7];
    /** 角加加加速度 / snap (rad/s⁴)，默认 [-200, +200] */
    double q_snap_min[7];
    double q_snap_max[7];

    RobotParams();
};

/**
 * SRS 规划器统一返回码：0=成功，负值=错误类型。
 * 供 srsInverseKinematics、optPsiSelector 等接口共用。
 */
enum class SrsResult : int {
    Ok = 0,

    // ---------- 逆解 (srsInverseKinematics) ----------
    /* 目标位姿超出工作空间：肩-腕距离 dSW 不满足 |dSE-dEW| <= dSW <= dSE+dEW，srsInverseKinematics8 无解 */
    IkTargetOutOfReach = -1,
    /* 8 组解中至少一组含 NaN/Inf，且过滤后无有效解；多由奇异构型或数值不稳定引起 */
    IkSolutionsNaNInf = -2,
    /* 8 组解均无数值异常，但全部超出关节限位 q_min/q_max，过滤后无有效解 */
    IkAllOutOfJointLimits = -3,
    /* 选出的最优解与上一时刻 q_prev 相比，某关节变化超过 1000°，视为异常跳变拒绝输出 */
    IkJointJump = -4,

    // ---------- 臂角优化 (optPsiSelector) ----------
    /* 当前目标位姿下 getFeasibleArmAngle 返回 0，无可行臂角区间 */
    OptNoFeasibleIntervals = -10,
    /* 在当前臂角 psi_t 处求逆解失败，无法得到梯度基准点，整段优化无法继续 */
    OptIkBaselineFailed = -11,
    /* 在 psi_t+deltaPsi 与 psi_t-deltaPsi 两处求逆解均失败，无法计算梯度 */
    OptIkProbeFailed = -12,
    /* 动态可行性验证失败，无法找到可行域 */
    OptDynamicInfeasible = -13,
};

class SrsPlanner {
public:
    // 设置参数（如果需要覆盖默认值）
    auto setParams(const RobotParams& params) -> void ;

    /** @brief 当前机器人参数（含关节与运动学限幅） */
    auto getParams() const -> const RobotParams& { return params_; }

    /**
     * @brief 将当前机器人参数保存到 JSON 文件（几何连杆长度、关节限位、末端-工具变换及 pq）
     * @return 成功返回 true，写入失败返回 false
     */
    auto saveParams(const std::string& filename) const -> bool ;

    /**
     * @brief 从 JSON 文件加载机器人参数；文件中缺失的键保留内存中已有值（按键覆盖）
     * @return 成功返回 true，文件不存在或无法解析返回 false
     */
    auto loadParams(const std::string& filename) -> bool ;

    /**
     * @brief 写入默认 srs 配置文件（与 test/test_plan/test_srs_planner/demo_init_srs 一致）
     * @param basename 相对日志目录的文件名，默认 "srs_config.json"；完整路径由 rtb::io::getLogFilePath 拼接
     * @return 成功返回 true
     */
    static auto saveDefaultSrsConfigJson(const std::string& basename = "srs_config.json") -> bool ;

    // ------------------------------------------------------------------------
    // 末端 → 工具 变换（内存读写，不自动写文件；与 saveParams/loadParams 区分开）
    // pq 布局：[x,y,z,qw,qx,qy,qz]，与 rtb::math::pm2pq / pq2pm 一致
    // ------------------------------------------------------------------------

    /** @brief 设置 T_ee_to_tool（16 行优先），并自动更新 T_tool_to_ee 与两侧 pq */
    auto setEeToToolMatrix(const double* T_ee_to_tool) -> void ;
    /** @brief 拷贝当前 T_ee_to_tool 到 T_out（至少 16 维） */
    auto getEeToToolMatrix(double* T_out) const -> void ;

    /** @brief 设置 T_tool_to_ee（16 行优先），并自动更新 T_ee_to_tool 与两侧 pq */
    auto setToolToEeMatrix(const double* T_tool_to_ee) -> void ;
    /** @brief 拷贝当前 T_tool_to_ee 到 T_out（至少 16 维） */
    auto getToolToEeMatrix(double* T_out) const -> void ;

    /** @brief 由 pq 设置末端→工具，并更新矩阵与工具→末端 pq */
    auto setEeToToolPosePq(const double* pq) -> void ;
    /** @brief 拷贝末端→工具 pq 到 pq_out（至少 7 维） */
    auto getEeToToolPosePq(double* pq_out) const -> void ;

    /** @brief 由 pq 设置工具→末端，并更新矩阵与末端→工具 pq */
    auto setToolToEePosePq(const double* pq) -> void ;
    /** @brief 拷贝工具→末端 pq 到 pq_out（至少 7 维） */
    auto getToolToEePosePq(double* pq_out) const -> void ;

    // ========================================================================
    // 核心算法接口
    // ========================================================================

    /**
     * @brief 获取所有 8 组逆解 (用于调试)
     * @param psi 臂角 (rad)
     * @param T_local 目标位姿 (4×4 行优先，长度至少 16)
     * @param solutions 输出 8 组解，布局 solutions[i*7+j] 为第 i 组第 j 关节，长度至少 56
     * @return 解的数量 (0 或 8)
     */
    auto calculateAllIKSolutions(double psi, const double* T_local, double* solutions) -> int ;

    /**
     * @brief 获取可行臂角区间
     * @param T_local 目标位姿 (4×4 行优先，长度至少 16)
     * @param out_intervals [输出] 可行臂角区间列表（预分配）
     * @return 区间数量
     */
    auto getFeasibleArmAngle(const double* T_local, IntervalArray& out_intervals) -> size_t ;
    /**
     * @brief 判断给定臂角是否处于目标位姿对应的可行域中
     * @param T_local 目标位姿 (4×4 行优先，长度至少 16)
     * @param psi 臂角 (rad)
     * @return true=在可行域内；false=不在可行域内
     */
    auto isArmAngleFeasible(const double* T_local, double psi) -> bool ;

    /**
     * @brief 判断给定关节角计算得到的当前臂角是否处于目标位姿对应的可行域中
     * @param T_local 目标位姿 (4×4 行优先，长度至少 16)
     * @param q 关节角 (7 维, rad)
     * @param ref_vec_local 参考平面法向量 (3 维)，可为 nullptr 表示 {0,0,1}
     * @return true=在可行域内；false=不在可行域内
     */
    auto isJointFeasible(const double* T_local, const double* q, const double* ref_vec_local = nullptr) -> bool ;

    /** @brief 仅 Case1 可行区间（调试/对比用） */
    auto getFeasibleArmAngleCase1(const double* T_local, IntervalArray& out_intervals) -> void ;
    /** @brief 仅 Case2 可行区间（调试/对比用） */
    auto getFeasibleArmAngleCase2(const double* T_local, IntervalArray& out_intervals) -> void ;

    /**
     * @brief 最优臂角规划 (Optimal Arm Angle Selector) - 简化版本
     * @param psi_t 上一时刻臂角 (rad)
     * @param q_t 上一时刻关节角 (7 维)
     * @param T_t1 当前时刻目标位姿 (16 维行优先)
     * @param psi_out [输出] 规划出的最优臂角
     * @return SrsResult::Ok=成功，负值见 SrsResult 枚举
     */
    auto optPsiSelector(double psi_t, const double* q_t, const double* T_t1, double& psi_out) -> SrsResult ;

    /**
     * @brief 最优臂角规划 (Optimal Arm Angle Selector) - 完整版本
     */
    auto optPsiSelector(double psi_t, const double* q_t, const double* T_t1, double& psi_out,
        IntervalArray& feasible_intervals, size_t& num_intervals, double& bestpsi_alter) -> SrsResult ;

    /**
     * @brief 最优臂角规划 (Optimal Arm Angle Selector) - 时空协同优化版本
     *
     * @param psi_t 上一拍臂角 (psi_prev)
     * @param q_prev 上一拍关节位置 (t-1时刻)
     * @param q_prev2 上上拍关节位置 (t-2时刻) -> 用于二阶差分计算加速度
     * @param T_t1 当前目标位姿
     * @param dt 控制周期 (例如 0.004)
     * @param v_limit 各关节速度极限数组 (正数, rad/s)
     * @param a_limit 各关节加速度极限数组 (正数, rad/s^2)
     * @param psi_out 输出：当前拍最优臂角
     * @param feasible_intervals 可行臂角区间列表
     * @param num_intervals 可行臂角区间数量
     * @param bestpsi_alter 最优臂角
     * @return SrsResult
     */
    auto optPsiSelector2(double psi_t, const double* q_prev, const double* q_prev2, const double* T_t1, double dt,
        const double* v_limit, const double* a_limit, double& psi_out, IntervalArray& feasible_intervals,
        size_t& num_intervals, double& bestpsi_alter) -> SrsResult ;

    /**
     * @brief 最优臂角规划 (Optimal Arm Angle Selector) - 纯空间居中 + 动态软引导版
     *
     * @param psi_t 上一拍臂角 (psi_prev)
     * @param q_prev 上一拍关节位置 (t-1时刻)
     * @param q_prev2 上上拍关节位置 (t-2时刻) -> 用于二阶差分计算加速度
     * @param T_t1 当前目标位姿
     * @param dt 控制周期 (例如 0.004)
     * @param v_limit 各关节速度极限数组 (正数, rad/s)
     * @param a_limit 各关节加速度极限数组 (正数, rad/s^2)
     * @param psi_out 输出：当前拍最优臂角
     * @param feasible_intervals 可行臂角区间列表
     * @param num_intervals 可行臂角区间数量
     * @param bestpsi_alter 最优臂角
     * @return SrsResult
     */
    auto optPsiSelector3(double psi_t, const double* q_prev, const double* q_prev2, const double* T_t1, double dt,
        const double* v_limit, const double* a_limit, double& psi_out, IntervalArray& feasible_intervals,
        size_t& num_intervals, double& bestpsi_alter) -> SrsResult ;

    /**
     * @brief 最优臂角规划 (Optimal Arm Angle Selector) - 臂角优化函数改进
     *
     * @param psi_t 上一拍臂角 (psi_prev)
     * @param q_prev 上一拍关节位置 (t-1时刻)
     * @param q_prev2 上上拍关节位置 (t-2时刻) -> 用于二阶差分计算加速度
     * @param T_t1 当前目标位姿
     * @param dt 控制周期 (例如 0.004)
     * @param v_limit 各关节速度极限数组 (正数, rad/s)
     * @param a_limit 各关节加速度极限数组 (正数, rad/s^2)
     * @param psi_out 输出：当前拍最优臂角
     * @param feasible_intervals 可行臂角区间列表
     * @param num_intervals 可行臂角区间数量
     * @param bestpsi_alter 最优臂角
     * @return SrsResult
     */
    auto optPsiSelector4(double psi_t, const double* q_prev, const double* q_prev2, const double* T_t1, double dt,
        const double* v_limit, const double* a_limit, double& psi_out, IntervalArray& feasible_intervals,
        size_t& num_intervals, double& bestpsi_alter) -> SrsResult ;

    /**
     * @brief 运动学逆解 (Inverse Kinematics) - 简化版本
     * @param psi 臂角 (rad)
     * @param T_local 末端位姿 (16 维行优先)
     * @param q_prev 上一时刻关节角 (7 维)
     * @param q_out [输出] 最优关节角 (至少 7 维)
     * @return SrsResult::Ok=成功，负值见 SrsResult 枚举
     */
    auto srsInverseKinematics(double psi, const double* T_local, const double* q_prev, double* q_out) -> SrsResult ;

    /**
     * @brief 运动学逆解 (Inverse Kinematics) - 带分支编号
     * @param branch_id [输出] 选中的解分支编号 (1-8)，无解时未定义
     */
    auto srsInverseKinematics(
        double psi, const double* T_local, const double* q_prev, double* q_out, int& branch_id) -> SrsResult ;

    /**
     * @brief SRS 运动学逆解，输出 8 组解
     * @param T 末端位姿 (16 维行优先)
     * @param q_ref 参考关节角 (7 维)
     * @param solutions 输出 8 组解，布局 solutions[i*7+j]，长度至少 56
     * @return 解的数量 (0 或 8)
     */
    auto srsInverseKinematics8(double psi, const double* T, const double* q_ref, double* solutions) -> int ;

    // ========================================================================
    // 验证与辅助工具
    // ========================================================================

    /**
     * @brief 正运动学 (Forward Kinematics)
     * @param q 关节角 (7 维, rad)
     * @param T_out 输出末端位姿矩阵 (4×4 行优先，至少 16)
     */
    auto forwardKinematics(const double* q, double* T_out) -> void ;

    /**
     * @brief 计算当前臂角
     * @param q 关节角 (7 维)
     * @param ref_vec_local 参考平面法向量 (3 维)，可为 nullptr 表示 {0,0,1}
     * @return 臂角 (rad)
     */
    auto calculateCurrentPsi(const double* q, const double* ref_vec_local = nullptr) -> double ;

    /**
     * @brief 计算当前关节角对应的雅可比条件数（基座坐标系几何雅可比）
     * @param q 当前 7 个关节角 (rad)
     * @return 条件数 = sigma_max/sigma_min；若奇异或计算失败返回无穷大
     */
    auto jacobianConditionNumber(const double* q) -> double ;

    /**
     * @brief 检测关节角是否超出限位
     * @param q 关节角 (7 维)
     * @return true=在限位内；false=超出限位
     */
    auto isInJointLimit(const double* q) -> bool ;

    /**
     * @brief 将关节角按关节上下限归一化到 [-1, 1]
     * @param q 输入关节角 (7 维)
     * @param q_norm_out 输出归一化结果 (7 维)，-1 对应 q_min，+1 对应 q_max
     */
    auto normalizeJointByLimits(const double* q, double* q_norm_out) const -> void ;

    SrsPlanner();

    /**
     * @brief 从 JSON 初始化：先将文件名传入 rtb::io::getLogFilePath 得到完整路径，再 loadParams
     * @param params_json_filename 如 "srs_robot.json"（与工程内其它 getLogFilePath 用法一致）
     */
    explicit SrsPlanner(const std::string& params_json_filename);

    ~SrsPlanner();

private:
    RobotParams params_;

    /** 末端(法兰)到工具坐标系 4×4，默认 create_transform_z(0.24, 0) */
    double T_ee_to_tool_[RTB_MAT4_SIZE]{};
    /** 工具到末端，由 T_ee_to_tool_ 求逆得到 */
    double T_tool_to_ee_[RTB_MAT4_SIZE]{};
    double pq_ee_to_tool_[RTB_PQ_SIZE]{};
    double pq_tool_to_ee_[RTB_PQ_SIZE]{};

    auto syncFromEeToToolMatrix() -> void ;
    auto syncFromToolToEeMatrix() -> void ;

    auto getFeasibleArmAngle_CaseX(const double* T, bool is_case1, IntervalArray& out_intervals) -> void ;

    // 区间计算（优化版：使用预分配缓冲区）
    auto calcTanIntervals(double q_l, double q_u, double an, double bn, double cn, double ad, double bd, double cd,
        IntervalArray& out_intervals) -> void ;
    auto calcCosIntervals(double q_limit_abs, double a, double b, double c, IntervalArray& out_intervals) -> void ;

    // 区间运算工具（优化版）
    auto comTwoInterval(const Interval& A, const Interval& B, Interval& out) -> bool ;
    auto comMultiInterval(const IntervalArray& A, const IntervalArray& B, IntervalArray& out_intervals) -> void ;
    auto mergeIntervals(IntervalArray& intervals) -> void ;
};

// ============================================================================
// 辅助工具函数
// ============================================================================

/**
 * @brief 创建带时间戳的文件名
 * @param prefix 前缀
 * @param extension 扩展名
 * @return 文件名 (例如: "prefix_20231027_153045.extension")
 */
auto createTimestampedFilename(const std::string& prefix, const std::string& extension) -> std::string ;

}  // namespace plan
}  // namespace rtb

#endif  // RTB_SRS_PLANNER_HPP
