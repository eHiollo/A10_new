#ifndef A10_GRIPPER_BRIDGE_HPP_
#define A10_GRIPPER_BRIDGE_HPP_

#include <atomic>
#include <cstdint>
#include <string>

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

/// 诊断状态：用来区分「夹爪没接上 / 读失败」和「夹爪正常但不响应」。
/// g_grip_service_active 为 false => /dev/ttyUSB0 没接上，夹爪命令会全部被忽略。
extern std::atomic<bool> g_grip_service_active;
/// 是否已从硬件拿到过可信的夹爪开度（读失败时不会被误当成 0 mm）。
extern std::atomic<bool> g_grip_position_valid;
/// 从 SET_EE_DELTA 收到的夹爪命令数。
extern std::atomic<std::uint64_t> g_grip_cmds_received;
/// 实际下发到舵机的目标位置次数（每次 >=0.3 mm 变化算一次）。
extern std::atomic<std::uint64_t> g_grip_writes;
/// 串口/舵机读失败次数。
extern std::atomic<std::uint64_t> g_grip_read_failures;

void set_vr_grip_cmd(double cmd);
void clear_vr_grip_cmd();
void sync_vr_grip_target_from_actual(double mm);
/// 实时安全：请求夹爪绝对开度（mm）。由 USB 服务线程执行，不在 RT 里访问串口。
void request_gripper_position_mm(double mm);

/// 非 RT 线程：积分 ``g_vr_grip_cmd`` 并调用 ``BusServo::set_gripper_position``。
/// ``gripper`` 为 nullptr 时立即返回并打印一次告警（arm-only 模式）。
void run_gripper_service_loop(BusServo* gripper);

/// 供 ``g_status`` 打印的一行诊断摘要。
std::string gripper_health_report();
}  // namespace a10_tcp

#endif
