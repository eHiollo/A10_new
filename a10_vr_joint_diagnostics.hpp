#pragma once

#include <cmath>
#include <cstdint>

namespace a10_tcp
{
enum JointDiagnosticFlag : std::uint32_t
{
    joint_diag_none = 0,
    joint_diag_non_finite = 1U << 0,
    joint_diag_command_step = 1U << 1,
    joint_diag_command_acceleration = 1U << 2,
    joint_diag_following_error = 1U << 3,
    joint_diag_actual_velocity = 1U << 4,
};

struct JointDiagnosticLimits
{
    double max_velocity_rad_s{0.0};
    double max_acceleration_rad_s2{0.0};
    double max_following_error_rad{0.0};
};

struct JointDiagnosticInput
{
    double previous_command_position_rad{0.0};
    double command_position_rad{0.0};
    double previous_command_velocity_rad_s{0.0};
    double actual_position_rad{0.0};
    double previous_actual_position_rad{0.0};
    double actual_sample_dt_s{0.0};
    bool has_previous_command_velocity{false};
    bool has_previous_actual_position{false};
};

struct JointDiagnosticResult
{
    std::uint32_t flags{joint_diag_none};
    double command_delta_rad{0.0};
    double command_velocity_rad_s{0.0};
    double command_acceleration_rad_s2{0.0};
    double actual_delta_rad{0.0};
    double actual_velocity_rad_s{0.0};
    double following_error_rad{0.0};

    bool abnormal() const { return flags != joint_diag_none; }
};

inline double shortest_angular_delta(double from_rad, double to_rad)
{
    constexpr double k_two_pi = 6.28318530717958647692;
    return std::remainder(to_rad - from_rad, k_two_pi);
}

inline JointDiagnosticResult evaluate_joint_diagnostic(
    const JointDiagnosticInput& input,
    const JointDiagnosticLimits& limits,
    double dt_s)
{
    JointDiagnosticResult result;
    const bool finite =
        std::isfinite(input.previous_command_position_rad)
        && std::isfinite(input.command_position_rad)
        && std::isfinite(input.previous_command_velocity_rad_s)
        && std::isfinite(input.actual_position_rad)
        && std::isfinite(input.previous_actual_position_rad)
        && std::isfinite(input.actual_sample_dt_s)
        && (!input.has_previous_actual_position || input.actual_sample_dt_s > 0.0)
        && std::isfinite(dt_s) && dt_s > 0.0;
    if (!finite)
    {
        result.flags |= joint_diag_non_finite;
        return result;
    }

    result.command_delta_rad =
        input.command_position_rad - input.previous_command_position_rad;
    result.command_velocity_rad_s = result.command_delta_rad / dt_s;
    if (input.has_previous_command_velocity)
    {
        result.command_acceleration_rad_s2 =
            (result.command_velocity_rad_s - input.previous_command_velocity_rad_s) / dt_s;
    }
    if (input.has_previous_actual_position && input.actual_sample_dt_s > 0.0)
    {
        result.actual_delta_rad = shortest_angular_delta(
            input.previous_actual_position_rad, input.actual_position_rad);
        result.actual_velocity_rad_s = result.actual_delta_rad / input.actual_sample_dt_s;
    }
    result.following_error_rad = shortest_angular_delta(
        input.actual_position_rad, input.command_position_rad);

    if (limits.max_velocity_rad_s > 0.0
        && std::abs(result.command_delta_rad) > limits.max_velocity_rad_s * dt_s)
    {
        result.flags |= joint_diag_command_step;
    }
    if (input.has_previous_command_velocity
        && limits.max_acceleration_rad_s2 > 0.0
        && std::abs(result.command_acceleration_rad_s2) > limits.max_acceleration_rad_s2)
    {
        result.flags |= joint_diag_command_acceleration;
    }
    if (limits.max_following_error_rad > 0.0
        && std::abs(result.following_error_rad) > limits.max_following_error_rad)
    {
        result.flags |= joint_diag_following_error;
    }
    if (input.has_previous_actual_position && input.actual_sample_dt_s > 0.0
        && limits.max_velocity_rad_s > 0.0
        && std::abs(result.actual_velocity_rad_s) > limits.max_velocity_rad_s)
    {
        result.flags |= joint_diag_actual_velocity;
    }
    return result;
}
}  // namespace a10_tcp
