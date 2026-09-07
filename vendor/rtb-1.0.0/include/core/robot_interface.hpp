/**
 * @file robot_mock.hpp
 * @brief 通用机器人接口 - 基于用户自定义函数的灵活接口
 *
 * 设计思路：
 * 1. 用户通过lambda函数注册具体实现
 * 2. 支持同一功能的多个实现（通过func_id区分）
 * 3. 简单、灵活、易于移植
 */

#ifndef ROBOT_MOCK_HPP
#define ROBOT_MOCK_HPP

#include <functional>
#include <iostream>
#include <map>
#include <string>

namespace rtb {
namespace core {
/**
 * @class RobotInterface
 * @brief 通用机器人控制接口
 *
 * 核心功能：
 * 1. 获取当前关节角度
 * 2. 获取当前末端位姿
 * 3. 运动学正解
 * 4. 运动学逆解
 * 5. 电机控制（下发关节期望角度）
 *
 * 用法示例：
 * @code
 * RobotInterface robot;
 *
 * // 注册获取关节角度的实现
 * robot.setGetJointsFunc(0, [&](double* joints) {
 *     for (int i = 0; i < 7; ++i) {
 *         joints[i] = controller()->motorPool()[i].actualPos();
 *     }
 * });
 *
 * // 调用
 * double joints[7];
 * robot.getJoints(0, joints);
 * @endcode
 */
class RobotInterface {
public:
    // ==================== 函数类型定义 ====================
    /**
     * @brief 获取关节角度的函数类型
     * @param func_id 功能ID
     * @param joints 输出：关节角度数组
     */
    using GetJointsFunc = std::function<void(double* joints)>;

    /**
     * @brief 获取末端位姿的函数类型
     * @param func_id 功能ID
     * @param pose 输出：末端位姿 [x,y,z,rx,ry,rz]
     */
    using GetPoseFunc = std::function<void(double* pose)>;

    /**
     * @brief 正运动学的函数类型
     * @param func_id 功能ID
     * @param joints 输入：关节角度数组 [rad]
     * @param ee_pos 输出：末端位姿 [x,y,z,rx,ry,rz] 或 4x4矩阵
     * @return true 成功, false 失败
     */
    using ForwardKinematicsFunc = std::function<bool(const double* joints, double* ee_pos)>;

    /**
     * @brief 逆运动学的函数类型
     * @param func_id 功能ID
     * @param ee_pos 输入：末端位姿 [x,y,z,rx,ry,rz] 或 4x4矩阵
     * @param joints 输出：关节角度数组 [rad]
     * @return true 成功, false 失败
     */
    using InverseKinematicsFunc = std::function<bool(const double* ee_pos, double* joints)>;

    /**
     * @brief 电机控制的函数类型
     * @param func_id 功能ID
     * @param joints 输入：目标关节角度
     */
    using SetJointsFunc = std::function<void(const double* joints)>;

    /**
     * @brief 通用参数设置的函数类型
     * @param func_id 功能ID
     * @param val 输入：参数值
     * @return true 成功, false 失败
     */
    using SetParamFunc = std::function<bool(const double* val)>;

    /**
     * @brief 通用参数获取的函数类型
     * @param func_id 功能ID
     * @param val 输出：参数值
     * @return true 成功, false 失败
     */
    using GetParamFunc = std::function<bool(double* val)>;

    /**
     * @brief 关节安全检查的函数类型
     * @param joints 输入：关节角度数组 [rad]
     * @param jvel 输入：关节速度数组 [rad/s]（可选，nullptr表示不检查）
     * @param jacc 输入：关节加速度数组 [rad/s^2]（可选，nullptr表示不检查）
     * @return true 安全, false 超限
     */
    using CheckJointsFunc = std::function<bool(const double* joints, const double* jvel, const double* jacc)>;

    /**
     * @brief 用户自定义安全检查的函数类型
     * @param input 输入：用户定义的输入数据
     * @return true 安全, false 不安全
     */
    using CheckSafeFunc = std::function<bool(const double* input)>;

    /**
     * @brief 关节角度限位的函数类型
     * @param joints 输入输出：关节角度数组，函数会直接修改超限的值
     */
    using JointsLimitFunc = std::function<void(double* joints)>;

    // ==================== 设置函数实现 ====================
    /**
     * @brief 注册"获取关节角度"的实现
     * @param func_id 功能ID（默认-1表示未设置）
     * @param func 函数实现
     */
    auto setGetJointsFunc(int func_id, GetJointsFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id << " for GetJointsFunc (must be >= 0)"
                      << std::endl;
            return;
        }
        get_joints_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered GetJointsFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"获取末端位姿"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setGetPoseFunc(int func_id, GetPoseFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id << " for GetPoseFunc (must be >= 0)"
                      << std::endl;
            return;
        }
        get_pose_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered GetPoseFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"正运动学"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setForwardKinematicsFunc(int func_id, ForwardKinematicsFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id
                      << " for ForwardKinematicsFunc (must be >= 0)" << std::endl;
            return;
        }
        fk_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered ForwardKinematicsFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"逆运动学"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setInverseKinematicsFunc(int func_id, InverseKinematicsFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id
                      << " for InverseKinematicsFunc (must be >= 0)" << std::endl;
            return;
        }
        ik_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered InverseKinematicsFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"电机控制"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setSetJointsFunc(int func_id, SetJointsFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id << " for SetJointsFunc (must be >= 0)"
                      << std::endl;
            return;
        }
        set_joints_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered SetJointsFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"通用参数设置"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setSetParamFunc(int func_id, SetParamFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id << " for SetParamFunc (must be >= 0)"
                      << std::endl;
            return;
        }
        set_param_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered SetParamFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"通用参数获取"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setGetParamFunc(int func_id, GetParamFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id << " for GetParamFunc (must be >= 0)"
                      << std::endl;
            return;
        }
        get_param_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered GetParamFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"关节安全检查"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setCheckJointsFunc(int func_id, CheckJointsFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id
                      << " for CheckJointsFunc (must be >= 0)" << std::endl;
            return;
        }
        check_joints_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered CheckJointsFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"用户自定义安全检查"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setCheckSafeFunc(int func_id, CheckSafeFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id << " for CheckSafeFunc (must be >= 0)"
                      << std::endl;
            return;
        }
        check_safe_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered CheckSafeFunc with ID " << func_id << std::endl;
    }

    /**
     * @brief 注册"关节角度限位"的实现
     * @param func_id 功能ID
     * @param func 函数实现
     */
    auto setJointsLimitFunc(int func_id, JointsLimitFunc func) -> void {
        if (func_id < 0) {
            std::cerr << "[RobotInterface] Warning: Invalid func_id " << func_id
                      << " for JointsLimitFunc (must be >= 0)" << std::endl;
            return;
        }
        joints_limit_funcs_[func_id] = func;
        std::cout << "[RobotInterface] Registered JointsLimitFunc with ID " << func_id << std::endl;
    }

    // ==================== 调用接口 ====================
    /**
     * @brief 获取关节角度
     * @param func_id 功能ID
     * @param joints 输出：关节角度数组
     */
    auto getJoints(int func_id, double* joints) -> void {
        auto it = get_joints_funcs_.find(func_id);
        if (it != get_joints_funcs_.end()) {
            it->second(joints);
        } else {
            warn("getJoints", func_id);
        }
    }

    /**
     * @brief 获取末端位姿
     * @param func_id 功能ID
     * @param pose 输出：末端位姿
     */
    auto getPose(int func_id, double* pose) -> void {
        auto it = get_pose_funcs_.find(func_id);
        if (it != get_pose_funcs_.end()) {
            it->second(pose);
        } else {
            warn("getPose", func_id);
        }
    }

    /**
     * @brief 正运动学
     * @param func_id 功能ID
     * @param joints 输入：关节角度
     * @param ee_pos 输出：末端位姿
     * @return true 成功, false 失败
     */
    auto forwardKinematics(int func_id, const double* joints, double* ee_pos) -> bool {
        auto it = fk_funcs_.find(func_id);
        if (it != fk_funcs_.end()) {
            return it->second(joints, ee_pos);
        } else {
            warn("forwardKinematics", func_id);
            return false;
        }
    }

    /**
     * @brief 逆运动学
     * @param func_id 功能ID
     * @param ee_pos 输入：末端位姿
     * @param joints 输出：关节角度
     * @return true 成功, false 失败
     */
    auto inverseKinematics(int func_id, const double* ee_pos, double* joints) -> bool {
        auto it = ik_funcs_.find(func_id);
        if (it != ik_funcs_.end()) {
            return it->second(ee_pos, joints);
        } else {
            warn("inverseKinematics", func_id);
            return false;
        }
    }

    /**
     * @brief 电机控制（下发关节期望角度）
     * @param func_id 功能ID
     * @param joints 输入：目标关节角度
     */
    auto setJoints(int func_id, const double* joints) -> void {
        auto it = set_joints_funcs_.find(func_id);
        if (it != set_joints_funcs_.end()) {
            it->second(joints);
        } else {
            warn("setJoints", func_id);
        }
    }

    /**
     * @brief 通用参数设置
     * @param func_id 功能ID
     * @param val 输入：参数值
     * @return true 成功, false 失败
     */
    auto setParam(int func_id, const double* val) -> bool {
        auto it = set_param_funcs_.find(func_id);
        if (it != set_param_funcs_.end()) {
            return it->second(val);
        } else {
            warn("setParam", func_id);
            return false;
        }
    }

    /**
     * @brief 通用参数获取
     * @param func_id 功能ID
     * @param val 输出：参数值
     * @return true 成功, false 失败
     */
    auto getParam(int func_id, double* val) -> bool {
        auto it = get_param_funcs_.find(func_id);
        if (it != get_param_funcs_.end()) {
            return it->second(val);
        } else {
            warn("getParam", func_id);
            return false;
        }
    }

    /**
     * @brief 关节安全检查
     * @param func_id 功能ID
     * @param joints 输入：关节角度数组
     * @param jvel 输入：关节速度数组（可选，nullptr表示不检查）
     * @param jacc 输入：关节加速度数组（可选，nullptr表示不检查）
     * @return true 安全, false 超限
     */
    auto checkJoints(int func_id, const double* joints, const double* jvel = nullptr, const double* jacc = nullptr) -> bool {
        auto it = check_joints_funcs_.find(func_id);
        if (it != check_joints_funcs_.end()) {
            return it->second(joints, jvel, jacc);
        } else {
            warn("checkJoints", func_id);
            return true;  // 默认返回true（安全），避免误报
        }
    }

    /**
     * @brief 用户自定义安全检查
     * @param func_id 功能ID
     * @param input 输入：用户定义的输入数据
     * @return true 安全, false 不安全
     */
    auto checkSafe(int func_id, const double* input) -> bool {
        auto it = check_safe_funcs_.find(func_id);
        if (it != check_safe_funcs_.end()) {
            return it->second(input);
        } else {
            warn("checkSafe", func_id);
            return true;  // 默认返回true（安全），避免误报
        }
    }

    /**
     * @brief 关节角度限位（直接修改输入数组）
     * @param func_id 功能ID
     * @param joints 输入输出：关节角度数组
     */
    auto jointsLimit(int func_id, double* joints) -> void {
        auto it = joints_limit_funcs_.find(func_id);
        if (it != joints_limit_funcs_.end()) {
            it->second(joints);
        } else {
            warn("jointsLimit", func_id);
        }
    }

    // ==================== 辅助功能 ====================
    /**
     * @brief 检查指定func_id的函数是否已注册（模板版本，直接传入类型）
     * @tparam FuncType 函数类型（GetJointsFunc, GetPoseFunc等）
     * @param func_id 功能ID
     * @return true 已注册, false 未注册
     */
    template <typename FuncType>
    auto isFuncRegistered(int func_id) const -> bool {
        if constexpr (std::is_same_v<FuncType, GetJointsFunc>) {
            return get_joints_funcs_.find(func_id) != get_joints_funcs_.end();
        } else if constexpr (std::is_same_v<FuncType, GetPoseFunc>) {
            return get_pose_funcs_.find(func_id) != get_pose_funcs_.end();
        } else if constexpr (std::is_same_v<FuncType, ForwardKinematicsFunc>) {
            return fk_funcs_.find(func_id) != fk_funcs_.end();
        } else if constexpr (std::is_same_v<FuncType, InverseKinematicsFunc>) {
            return ik_funcs_.find(func_id) != ik_funcs_.end();
        } else if constexpr (std::is_same_v<FuncType, SetJointsFunc>) {
            return set_joints_funcs_.find(func_id) != set_joints_funcs_.end();
        } else {
            return false;
        }
    }

    /**
     * @brief 打印所有已注册的函数
     */
    auto printRegisteredFuncs() const -> void {
        std::cout << "\n========== Registered Functions ==========" << std::endl;

        std::cout << "GetJoints: ";
        for (const auto& pair : get_joints_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "GetPose: ";
        for (const auto& pair : get_pose_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "ForwardKinematics: ";
        for (const auto& pair : fk_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "InverseKinematics: ";
        for (const auto& pair : ik_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "SetJoints: ";
        for (const auto& pair : set_joints_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "SetParam: ";
        for (const auto& pair : set_param_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "GetParam: ";
        for (const auto& pair : get_param_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "CheckJoints: ";
        for (const auto& pair : check_joints_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "CheckSafe: ";
        for (const auto& pair : check_safe_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "JointsLimit: ";
        for (const auto& pair : joints_limit_funcs_) {
            std::cout << pair.first << " ";
        }
        std::cout << std::endl;

        std::cout << "==========================================" << std::endl;
    }

private:
    // 存储各类函数的映射表（func_id -> 函数实现）
    std::map<int, GetJointsFunc> get_joints_funcs_;
    std::map<int, GetPoseFunc> get_pose_funcs_;
    std::map<int, ForwardKinematicsFunc> fk_funcs_;
    std::map<int, InverseKinematicsFunc> ik_funcs_;
    std::map<int, SetJointsFunc> set_joints_funcs_;
    std::map<int, SetParamFunc> set_param_funcs_;
    std::map<int, GetParamFunc> get_param_funcs_;
    std::map<int, CheckJointsFunc> check_joints_funcs_;
    std::map<int, CheckSafeFunc> check_safe_funcs_;
    std::map<int, JointsLimitFunc> joints_limit_funcs_;

    // 辅助：输出警告信息
    auto warn(const std::string& func_name, int func_id) const -> void {
        std::cerr << "[RobotInterface] ⚠️  Warning: Function '" << func_name << "' with ID " << func_id
                  << " is not registered!" << std::endl;
        std::cerr << "  → Please register it using set" << func_name << "Func(" << func_id << ", ...)" << std::endl;
    }
};

}  // namespace core
}  // namespace rtb

#endif  // ROBOT_MOCK_HPP
