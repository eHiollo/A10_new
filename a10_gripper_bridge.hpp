#ifndef A10_GRIPPER_BRIDGE_HPP_
#define A10_GRIPPER_BRIDGE_HPP_

#include <atomic>
#include <cstdint>

class BusServo;

namespace a10_tcp
{
constexpr std::uint8_t k_gripper_servo_id = 10;
constexpr char k_gripper_type[] = "100mm";
constexpr std::uint16_t k_gripper_move_speed = 800;
constexpr double k_gripper_mm_min = 0.0;
constexpr double k_gripper_mm_max = 100.0;
constexpr double k_grip_cmd_deadzone = 0.05;

/// VR 摇杆夹爪速度指令 [-1, 1]；+1 张开，-1 闭合。
extern std::atomic<double> g_vr_grip_cmd;
/// 夹爪最大速度 mm/s（可由 vr 的 ``grip_vel`` 覆盖）。
extern std::atomic<double> g_vr_grip_vel_mm_s;
extern std::atomic<double> g_vr_grip_target_mm;
extern std::atomic<double> g_vr_grip_actual_mm;

void set_vr_grip_cmd(double cmd);
void clear_vr_grip_cmd();
void sync_vr_grip_target_from_actual(double mm);
/// 实时安全：请求夹爪绝对开度（mm）。由 USB 服务线程执行，不在 RT 里访问串口。
void request_gripper_position_mm(double mm);

/// 非 RT 线程：积分 ``g_vr_grip_cmd`` 并调用 ``BusServo::set_gripper_position``。
void run_gripper_service_loop(BusServo* gripper);
}  // namespace a10_tcp

#endif
