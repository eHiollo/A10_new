#include "a10_gripper_bridge.hpp"

#include "gripper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>

namespace a10_tcp
{
std::atomic<double> g_vr_grip_cmd{0.0};
std::atomic<double> g_vr_grip_vel_mm_s{60.0};
std::atomic<double> g_vr_grip_target_mm{0.0};
std::atomic<double> g_vr_grip_actual_mm{0.0};
std::atomic<bool> g_grip_pos_pending{false};

std::atomic<bool> g_grip_service_active{false};
std::atomic<bool> g_grip_position_valid{false};
std::atomic<std::uint64_t> g_grip_cmds_received{0};
std::atomic<std::uint64_t> g_grip_writes{0};
std::atomic<std::uint64_t> g_grip_read_failures{0};

void set_vr_grip_cmd(double cmd)
{
    cmd = std::clamp(cmd, -1.0, 1.0);
    if (std::abs(cmd) < k_grip_cmd_deadzone)
    {
        cmd = 0.0;
    }
    g_vr_grip_cmd.store(cmd, std::memory_order_release);
    g_grip_cmds_received.fetch_add(1, std::memory_order_relaxed);
}

void clear_vr_grip_cmd()
{
    g_vr_grip_cmd.store(0.0, std::memory_order_release);
}

void sync_vr_grip_target_from_actual(double mm)
{
    mm = std::clamp(mm, k_gripper_mm_min, k_gripper_mm_max);
    g_vr_grip_target_mm.store(mm, std::memory_order_release);
    g_vr_grip_actual_mm.store(mm, std::memory_order_release);
}

void request_gripper_position_mm(double mm)
{
    mm = std::clamp(mm, k_gripper_mm_min, k_gripper_mm_max);
    g_vr_grip_target_mm.store(mm, std::memory_order_release);
    g_grip_pos_pending.store(true, std::memory_order_release);
}

void run_gripper_service_loop(BusServo* gripper)
{
    constexpr auto k_period = std::chrono::milliseconds(33);
    constexpr double k_dt = 0.033;
    constexpr double k_send_mm_eps = 0.3;
    constexpr int k_idle_read_interval = 15;
    constexpr int k_read_fail_warn_after = 20; // 连续失败约 10 s 后告警

    // 夹爪没接上时，原实现是「静默空转」：夹爪命令全部被丢弃，
    // 现象就是「机械臂能动、夹爪毫无反应」，且没有任何日志可查。
    // 这里改为启动时明确告警。
    if (gripper == nullptr)
    {
        g_grip_service_active.store(false, std::memory_order_release);
        std::fprintf(stderr,
                     "[gripper] ARM-ONLY MODE: /dev/ttyUSB0 未连接，夹爪命令会被全部忽略"
                     "（机械臂仍正常）。\n");
        std::fflush(stderr);
        return;
    }
    g_grip_service_active.store(true, std::memory_order_release);

    double target_mm = 0.0;
    double last_sent_mm = 0.0;
    double actual_mm = 0.0;
    bool have_target = false;
    // g_open/g_close/g_move 是显式绝对位置请求，即使目标与上次相同也必须下发，
    // 否则在“位置未知”时会被 0.3 mm 变化阈值挡掉而静默不动。
    bool send_required = false;
    int idle_ticks = 0;
    int consecutive_read_failures = 0;

    // 只在读取成功时播种。get_position_mm 读失败会返回 0，而 0 同时表示
    // 「完全闭合」，直接采信会让后续闭合指令被判为「无变化」而不下发。
    if (gripper->try_get_position_mm(k_gripper_servo_id, k_gripper_type, actual_mm))
    {
        target_mm = actual_mm;
        last_sent_mm = actual_mm;
        have_target = true;
        g_grip_position_valid.store(true, std::memory_order_release);
        sync_vr_grip_target_from_actual(actual_mm);
    }
    else
    {
        g_grip_read_failures.fetch_add(1, std::memory_order_relaxed);
        std::fprintf(stderr, "[gripper] 首次读位置失败，暂缓播种（不假定为 0 mm）\n");
        std::fflush(stderr);
    }

    while (true)
    {
        std::this_thread::sleep_for(k_period);

        const bool pos_pending = g_grip_pos_pending.exchange(false, std::memory_order_acq_rel);
        const double cmd = g_vr_grip_cmd.load(std::memory_order_acquire);
        const double vmax = g_vr_grip_vel_mm_s.load(std::memory_order_acquire);

        if (pos_pending)
        {
            target_mm = g_vr_grip_target_mm.load(std::memory_order_acquire);
            have_target = true;
            send_required = true;
        }
        else if (std::abs(cmd) >= 1e-9)
        {
            if (!have_target)
            {
                // 起点未知时先读真实开度；读不到就不动，避免盲积分。
                if (gripper->try_get_position_mm(k_gripper_servo_id, k_gripper_type, actual_mm))
                {
                    target_mm = actual_mm;
                    last_sent_mm = actual_mm;
                    have_target = true;
                    g_grip_position_valid.store(true, std::memory_order_release);
                }
                else
                {
                    g_grip_read_failures.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
            }
            target_mm += cmd * vmax * k_dt;
        }

        target_mm = std::clamp(target_mm, k_gripper_mm_min, k_gripper_mm_max);
        g_vr_grip_target_mm.store(target_mm, std::memory_order_release);

        if (pos_pending || std::abs(cmd) >= 1e-9)
        {
            idle_ticks = 0;
            if (send_required || std::abs(target_mm - last_sent_mm) >= k_send_mm_eps)
            {
                gripper->set_gripper_position(
                    k_gripper_servo_id, k_gripper_type, target_mm, k_gripper_move_speed);
                last_sent_mm = target_mm;
                send_required = false;
                g_grip_writes.fetch_add(1, std::memory_order_relaxed);
            }
            g_vr_grip_actual_mm.store(target_mm, std::memory_order_release);
            continue;
        }

        ++idle_ticks;
        if (idle_ticks >= k_idle_read_interval)
        {
            idle_ticks = 0;
            if (gripper->try_get_position_mm(k_gripper_servo_id, k_gripper_type, actual_mm))
            {
                consecutive_read_failures = 0;
                g_vr_grip_actual_mm.store(actual_mm, std::memory_order_release);
                target_mm = actual_mm;
                last_sent_mm = actual_mm;
                have_target = true;
                g_grip_position_valid.store(true, std::memory_order_release);
                g_vr_grip_target_mm.store(target_mm, std::memory_order_release);
            }
            else
            {
                // 保留上次已知目标，不要退回 0。
                g_grip_read_failures.fetch_add(1, std::memory_order_relaxed);
                g_grip_position_valid.store(false, std::memory_order_release);
                if (++consecutive_read_failures == k_read_fail_warn_after)
                {
                    std::fprintf(stderr,
                                 "[gripper] 位置读取连续失败，保留上次目标 %.1f mm\n",
                                 target_mm);
                    std::fflush(stderr);
                }
            }
        }
    }
}

std::string gripper_health_report()
{
    std::ostringstream os;
    os << "service="
       << (g_grip_service_active.load(std::memory_order_acquire) ? "active"
                                                                 : "ARM_ONLY(no /dev/ttyUSB0)")
       << " position_valid="
       << (g_grip_position_valid.load(std::memory_order_acquire) ? "yes" : "no")
       << " cmds_rx=" << g_grip_cmds_received.load(std::memory_order_relaxed)
       << " servo_writes=" << g_grip_writes.load(std::memory_order_relaxed)
       << " read_failures=" << g_grip_read_failures.load(std::memory_order_relaxed)
       << " target_mm=" << g_vr_grip_target_mm.load(std::memory_order_acquire)
       << " actual_mm=" << g_vr_grip_actual_mm.load(std::memory_order_acquire)
       << " cmd=" << g_vr_grip_cmd.load(std::memory_order_acquire);
    return os.str();
}
}  // namespace a10_tcp
