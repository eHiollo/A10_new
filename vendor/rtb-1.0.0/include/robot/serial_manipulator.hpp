#ifndef MY_ROBOTICS_SERIAL_MANIPULATOR_HPP
#define MY_ROBOTICS_SERIAL_MANIPULATOR_HPP

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <vector>

namespace rtb {
namespace robot {

// 关节类型枚举：支持旋转关节和平移（直线）关节
enum class JointType {
    REVOLUTE,
    PRISMATIC
};

// DH 参数及关节物理约束结构体
struct DHParam {
    JointType type = JointType::REVOLUTE;

    // 运动学几何参数
    double a = 0.0;
    double alpha = 0.0;
    double d = 0.0;      // 若为平移关节，此值为初始 offset
    double theta = 0.0;  // 若为旋转关节，此值为初始 offset

    // 位置约束 (q 表示角度或位移)
    double q_min = -std::numeric_limits<double>::infinity();
    double q_max = std::numeric_limits<double>::infinity();

    // 速度约束 (Velocity)
    double v_min = -std::numeric_limits<double>::infinity();
    double v_max = std::numeric_limits<double>::infinity();

    // 加速度约束 (Acceleration)
    double a_min = -std::numeric_limits<double>::infinity();
    double a_max = std::numeric_limits<double>::infinity();

    // 加加速度约束 (Jerk)
    double j_min = -std::numeric_limits<double>::infinity();
    double j_max = std::numeric_limits<double>::infinity();
};

class SerialManipulator {
public:
    // --- 构造与生命周期管理 ---
    explicit SerialManipulator(const std::vector<DHParam>& dh_table);
    ~SerialManipulator();

    SerialManipulator(const SerialManipulator&) = delete;
    auto operator=(const SerialManipulator&) -> SerialManipulator& = delete;

    SerialManipulator(SerialManipulator&&) noexcept;
    auto operator=(SerialManipulator&&) noexcept -> SerialManipulator&;

    // --- Getters & Setters ---
    auto getDoF() const -> int;
    auto getDHTable() const -> std::vector<DHParam>;

    // 引入世界坐标系到基坐标系的变换 T_world_base
    auto setBaseTransform(const Eigen::Matrix4d& T_world_base) -> void;
    auto getBaseTransform() const -> Eigen::Matrix4d;

    // 动态更新关节约束 (返回 bool 表示输入维度是否合法)
    auto setPositionLimits(const std::vector<double>& min_vals, const std::vector<double>& max_vals) -> bool;
    auto setVelocityLimits(const std::vector<double>& min_vals, const std::vector<double>& max_vals) -> bool;
    auto setAccelerationLimits(const std::vector<double>& min_vals, const std::vector<double>& max_vals) -> bool;
    auto setJerkLimits(const std::vector<double>& min_vals, const std::vector<double>& max_vals) -> bool;

    // --- 核心运动学接口 ---

    // 计算末端执行器在世界坐标系下的位姿
    auto computeFK(const Eigen::VectorXd& q) -> Eigen::Matrix4d;

    // 获取指定关节坐标系在世界坐标系下的位姿矩阵 (joint_index: 0 到 DoF-1)
    // 非常适用于碰撞检测或可视化绘制
    auto computeJointPose(const Eigen::VectorXd& q, int joint_index) -> Eigen::Matrix4d;

    // 计算当前构型下的空间雅可比矩阵 (6 x DoF)，基于世界坐标系
    auto computeJacobian(const Eigen::VectorXd& q) -> Eigen::MatrixXd;

    // --- 逆运动学接口 ---

    // 阻尼最小二乘法 (DLS)
    auto inverseKinematicsDLS(const Eigen::Matrix4d& target_pose_world,
        const Eigen::VectorXd& current_q,
        Eigen::VectorXd& out_q,
        double tolerance = 1e-4,
        int max_iter = 100,
        double lambda = 0.1) -> bool;

    // 冗余机械臂伪逆法 (带零空间投影)
    auto inverseKinematicsRedundant(const Eigen::Matrix4d& target_pose_world,
        const Eigen::VectorXd& current_q,
        const Eigen::VectorXd& q_nullspace_velocity,
        Eigen::VectorXd& out_q,
        double tolerance = 1e-4,
        int max_iter = 100) -> bool;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

}  // namespace robot
}  // namespace rtb

#endif  // MY_ROBOTICS_SERIAL_MANIPULATOR_HPP