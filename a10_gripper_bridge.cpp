#include "a10_gripper_bridge.hpp"

#include "gripper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace a10_tcp
{
std::atomic<double> g_vr_grip_cmd{0.0};
std::atomic<double> g_vr_grip_vel_mm_s{60.0};
std::atomic<double> g_vr_grip_target_mm{0.0};
std::atomic<double> g_vr_grip_actual_mm{0.0};
std::atomic<bool> g_grip_pos_pending{false};

void set_vr_grip_cmd(double cmd)
{
    cmd = std::clamp(cmd, -1.0, 1.0);
    if (std::abs(cmd) < k_grip_cmd_deadzone)
    {
        cmd = 0.0;
    }
    g_vr_grip_cmd.store(cmd, std::memory_order_release);
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

    double target_mm = 0.0;
    double last_sent_mm = -1.0;
    int idle_ticks = 0;
    if (gripper != nullptr)
    {
        target_mm = gripper->get_position_mm(k_gripper_servo_id, k_gripper_type);
        last_sent_mm = target_mm;
        sync_vr_grip_target_from_actual(target_mm);
    }

    while (true)
    {
        std::this_thread::sleep_for(k_period);
        if (gripper == nullptr)
        {
            continue;
        }

        const bool pos_pending = g_grip_pos_pending.exchange(false, std::memory_order_acq_rel);
        const double cmd = g_vr_grip_cmd.load(std::memory_order_acquire);
        const double vmax = g_vr_grip_vel_mm_s.load(std::memory_order_acquire);
        if (pos_pending)
        {
            target_mm = g_vr_grip_target_mm.load(std::memory_order_acquire);
        }
        else
        {
            target_mm += cmd * vmax * k_dt;
        }
        target_mm = std::clamp(target_mm, k_gripper_mm_min, k_gripper_mm_max);
        g_vr_grip_target_mm.store(target_mm, std::memory_order_release);

        if (pos_pending || std::abs(cmd) >= 1e-9)
        {
            idle_ticks = 0;
            if (std::abs(target_mm - last_sent_mm) >= k_send_mm_eps)
            {
                gripper->set_gripper_position(
                    k_gripper_servo_id, k_gripper_type, target_mm, k_gripper_move_speed);
                last_sent_mm = target_mm;
            }
            g_vr_grip_actual_mm.store(target_mm, std::memory_order_release);
            continue;
        }

        ++idle_ticks;
        if (idle_ticks >= k_idle_read_interval)
        {
            idle_ticks = 0;
            const double actual = gripper->get_position_mm(k_gripper_servo_id, k_gripper_type);
            g_vr_grip_actual_mm.store(actual, std::memory_order_release);
            target_mm = actual;
            last_sent_mm = actual;
            g_vr_grip_target_mm.store(target_mm, std::memory_order_release);
        }
    }
}
}  // namespace a10_tcp
