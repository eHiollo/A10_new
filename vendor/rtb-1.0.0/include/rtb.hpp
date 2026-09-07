
/* ===========================================================================
 *  @file    rtb.hpp
 *  @brief   Robotics ToolBox (RTB) 统一头文件
 *  @version 1.0.0
 *  @date    2025
 *
 *  @section 简介
 *  RTB（Robotics ToolBox）为机器人控制与运动规划提供核心算法模块，
 *  统一引入所有常用功能，便于开发与集成。
 *
 *  @section 模块概览
 *    - core  : 基础数学工具、宏定义、机器人接口与模拟
 *    - force : 六维力传感器标定、重力补偿、导纳控制
 *    - plan  : 轨迹跟随、单轴跟随、SRS冗余机械臂运动规划
 *
 *  @section 主要功能
 *    - 向量、矩阵、四元数等常用数学运算
 *    - 文件路径、时间戳、插值等实用工具
 *    - 机器人接口与模拟（支持无硬件开发）
 *    - 力传感器标定与补偿、导纳控制算法
 *    - 平滑轨迹跟随、单轴跟随、SRS七自由度冗余机械臂规划
 *
 *  @section 使用示例
 *    @code{.cpp}
 *    #include <rtb.hpp>
 *    // 力传感器标定
 *    rtb::force::ForceSensorCalibrator calibrator;
 *    // 导纳控制器
 *    rtb::force::AdmittanceController controller;
 *    // 轨迹跟随
 *    rtb::plan::MoveFollower follower;
 *    // SRS规划器
 *    rtb::plan::SrsPlanner planner;
 *    // 数学工具
 *    using namespace rtb::math;
 *    double n = vec3_norm(vec);
 *    @endcode
 *
 *  @note 所有模块均在 rtb 命名空间下，便于统一管理和调用。
 *
 *  @section 模块文件说明
 *    - rtb::math
 *        - math/math.hpp    : 向量、矩阵、四元数、旋转等数学运算
 *        - math/fast_math.hpp : 快速数学运算
 *    - rtb::core
 *        - core/utils.hpp   : 文件、时间、插值等实用工具
 *        - core/macro.hpp   : 通用宏定义
 *        - core/robot_interface.hpp : 机器人接口与模拟
 *    - rtb::force
 *        - admittance_controller.hpp    : 导纳控制器
 *        - force_sensor_calibrator.hpp  : 六维力传感器标定与重力补偿
 *    - rtb::plan
 *        - move_follower.hpp            : 平滑轨迹跟随
 *        - single_axis_follower.hpp     : 单轴跟随
 *        - srs_planner.hpp              : SRS冗余机械臂运动规划
 *    - rtb::control
 *        - control/pid.hpp              : 通用PID控制器（含抗积分饱和与输出限幅）
 * ===========================================================================
 */

#ifndef RTB_HPP
#define RTB_HPP

// ============================================================================
// 版本信息
// ============================================================================
#define RTB_VERSION_MAJOR 1
#define RTB_VERSION_MINOR 0
#define RTB_VERSION_PATCH 0
#define RTB_VERSION "1.0.0"

// ============================================================================
// 核心模块
// ============================================================================

// 数学工具库 (基础依赖)
// #include "fw/mock_interface.hpp"
#include "fw/bind.hpp"
#include "fw/motion.hpp"
#include "fw/object.hpp"
#include "fw/sensor.hpp"
#include "fw/skill.hpp"
#include "fw/system.hpp"

#include "skill/demo_skill.hpp"
#include "skill/multi_axis_follow.hpp"

#include "core/macro.hpp"
#include "core/robot_interface.hpp"  // 机器人接口模块（用于开发与部署）
#include "core/utils.hpp"

#include "math/fast_math.hpp"
#include "math/math.hpp"

#include "io/rtb_logger.hpp"
#include "io/rtb_time.hpp"
#include "io/rtbio.hpp"
#include "io/utils_json.hpp"
#ifdef RTB_YAML_ENABLED
#include "io/utils_yaml.hpp"
#endif

// 控制算法库
#include "control/discrete_plant.hpp"
#include "control/lqr.hpp"
#include "control/pid.hpp"
#include "control/zoh.hpp"

// 数字信号处理库
#include "signal/fft.hpp"
#include "signal/high_pass_filter.hpp"
#include "signal/kalman_filter.hpp"
#include "signal/kinematic_kalman_filter.hpp"
#include "signal/low_pass_filter.hpp"
#include "signal/luenberger_observer.hpp"
#include "signal/median_filter.hpp"
#include "signal/signal_generator.hpp"
#include "signal/tracking_differentiator.hpp"

// 力控制模块
#include "force/admittance_controller.hpp"
#include "force/contact_point_estimator.hpp"
#include "force/force_sensor_calibrator.hpp"

// 通用控制模块
#include "control/pid.hpp"

// 运动规划模块
#include "plan/SE3_follower.hpp"
#include "plan/input_interpolator.hpp"
#include "plan/input_smoother.hpp"
#include "plan/move_follower.hpp"
#include "plan/quintic_blender.hpp"
#ifdef RTB_OSQP_ENABLED
#include "plan/online_smoother.hpp"
#endif
#include "plan/single_axis_follower.hpp"
#include "plan/srs_planner.hpp"

// 机器人模块
#ifdef RTB_ROBOT_ENABLED
#include "robot/serial_manipulator.hpp"
#endif

/**
 * @namespace rtb
 * @brief RTB 库的根命名空间
 */
namespace rtb {

auto init() -> void ;
auto shutdown() -> void ;

}  // namespace rtb

#endif  // RTB_HPP
