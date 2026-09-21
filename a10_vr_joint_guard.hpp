#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace a10_tcp
{
// Joint-space envelope for the anchor driver. No SDK, allocation or I/O in step().
// Teleop limits are deliberately below the first six MotorConfig entries in
// kaanh.xml. The Sep 21 trace reached 3.224 rad/s actual wrist speed with a
// 2.985 rad/s command: the previous 5% margin did not cover servo overshoot.
class VrJointGuard
{
public:
    using Joints = std::array<double, 6>;
    static constexpr double dt = 0.002;
    static constexpr double acceleration = 0.20 * 17.453292519943293; // 200 deg/s^2
    static constexpr double position_limit = 3.0543261909900763;
    static constexpr double following_limit = 0.10;
    // 500 ms allows the <=225 ms ramp from rest to the teleop speed cap.
    // Only active demand counts; intentional braking must pass stop=true.
    static constexpr unsigned limited_cycle_limit = 250;

    static double speed_limit(std::size_t i)
    {
        return 0.25 * (i < 3 ? 2.6179938779914944 : 3.1415926535897931);
    }

    // Configure once before motion from MotorConfig, preserving asymmetric axes.
    bool set_position_limits(const Joints& minimum, const Joints& maximum)
    {
        for (std::size_t i = 0; i < 6; ++i)
            if (!std::isfinite(minimum[i]) || !std::isfinite(maximum[i])
                || minimum[i] >= maximum[i]) return false;
        minimum_ = minimum;
        maximum_ = maximum;
        return true;
    }
    double minimum(std::size_t i) const { return minimum_[i]; }
    double maximum(std::size_t i) const { return maximum_[i]; }

    struct Result
    {
        Joints position{};
        Joints velocity{};
        bool limited{false};
        double scale{1.0};
        const char* fault{"none"};
        int joint{0}; // 1-based; 0 means whole-arm or no fault.
    };

    void reset()
    {
        fault_ = "none";
        fault_joint_ = 0;
        limited_cycles_ = 0;
        fault_latched_ = false;
    }
    bool faulted() const { return fault_latched_; }
    const char* fault() const { return fault_; }

    // A safety fault is deliberately NOT cleared by release/new anchor packets.
    // Stop the plan, recover the drive, then restart with measured joint positions.
    void latch(const char* reason, int joint = 0)
    {
        if (!faulted()) { fault_ = reason; fault_joint_ = joint; fault_latched_ = true; }
    }

    static bool ready_to_anchor(const Joints& previous, const Joints& velocity,
                                const Joints& actual, const Joints& actual_velocity)
    {
        for (std::size_t i = 0; i < 6; ++i)
        {
            if (!std::isfinite(previous[i]) || !std::isfinite(velocity[i])
                || !std::isfinite(actual[i]) || !std::isfinite(actual_velocity[i])
                || std::abs(previous[i] - actual[i]) > 0.01
                || std::abs(velocity[i]) > 0.01 || std::abs(actual_velocity[i]) > 0.03)
                return false;
        }
        return true;
    }

    Result step(const Joints& previous, const Joints& velocity,
                const Joints& requested, const Joints& actual, bool stop = false)
    {
        Result out;
        Joints desired{};
        double lo = 0.0, hi = 1.0;
        const double dv = acceleration * dt;
        double max_speed = 0.0;
        for (double v : velocity) max_speed = std::max(max_speed, std::abs(v));
        for (std::size_t i = 0; i < 6; ++i)
        {
            // previous/velocity are accepted finite state, never raw IK output.
            if (!std::isfinite(actual[i]) || !std::isfinite(requested[i]))
                latch("joint_non_finite", static_cast<int>(i + 1));
            if (std::abs(previous[i] - actual[i]) > following_limit)
                latch("joint_following_error", static_cast<int>(i + 1));
            if (!stop && std::abs(requested[i] - previous[i]) > 0.10)
                latch("joint_ik_jump", static_cast<int>(i + 1));
            // Reserve the complete discrete braking distance before a joint limit.
            const double stopping_distance = std::abs(velocity[i]) * max_speed
                / (2.0 * acceleration) + std::abs(velocity[i]) * dt;
            const double available = velocity[i] < 0.0
                ? previous[i] - minimum_[i] : maximum_[i] - previous[i];
            if (requested[i] < minimum_[i] || requested[i] > maximum_[i]
                || actual[i] < minimum_[i] || actual[i] > maximum_[i]
                || available <= stopping_distance + 0.002)
                latch("joint_position_limit", static_cast<int>(i + 1));

            desired[i] = (requested[i] - previous[i]) / dt;
            // Find a COMMON scale satisfying all six speed and acceleration
            // intervals. Preserve IK's joint-step direction whenever feasible.
            const double lower = std::max(-speed_limit(i), velocity[i] - dv);
            const double upper = std::min(speed_limit(i), velocity[i] + dv);
            if (std::abs(desired[i]) > 1e-12 && std::isfinite(desired[i]))
            {
                const double a = lower / desired[i], b = upper / desired[i];
                lo = std::max(lo, std::min(a, b));
                hi = std::min(hi, std::max(a, b));
            }
            else if (lower > 0.0 || upper < 0.0) lo = 2.0;
        }
        const bool feasible = lo <= hi && hi >= 0.0;
        out.limited = !feasible || hi < 1.0 - 1e-10;
        limited_cycles_ = (!stop && out.limited) ? limited_cycles_ + 1 : 0;
        if (limited_cycles_ >= limited_cycle_limit) latch("joint_limit_persistent");
        const bool brake = stop || faulted() || !feasible;
        out.scale = brake ? 0.0 : hi;
        const double brake_scale = max_speed <= dv ? 0.0 : 1.0 - dv / max_speed;
        for (std::size_t i = 0; i < 6; ++i)
        {
            // Braking also respects acceleration; never snap a moving command to
            // measured feedback. At rest, faulted output stays at the last stop.
            out.velocity[i] = brake
                ? velocity[i] * brake_scale
                : hi * desired[i];
            out.position[i] = previous[i] + out.velocity[i] * dt;
        }
        out.fault = fault_;
        out.joint = fault_joint_;
        return out;
    }

private:
    Joints minimum_{{-position_limit, -position_limit, -position_limit,
                     -position_limit, -position_limit, -position_limit}};
    Joints maximum_{{position_limit, position_limit, position_limit,
                     position_limit, position_limit, position_limit}};
    const char* fault_{"none"};
    int fault_joint_{0};
    unsigned limited_cycles_{0};
    bool fault_latched_{false};
};
} // namespace a10_tcp
