#include "a10_vr_joint_diagnostics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
constexpr double k_dt = 0.002;

a10_tcp::JointDiagnosticLimits limits()
{
    return {2.0, 100.0, 0.5};
}

void test_normal_sample()
{
    a10_tcp::JointDiagnosticInput input;
    input.previous_command_position_rad = 0.1;
    input.command_position_rad = 0.102;
    input.previous_command_velocity_rad_s = 1.0;
    input.actual_position_rad = 0.101;
    input.previous_actual_position_rad = 0.099;
    input.actual_sample_dt_s = k_dt;
    input.has_previous_command_velocity = true;
    input.has_previous_actual_position = true;
    const auto result = a10_tcp::evaluate_joint_diagnostic(input, limits(), k_dt);
    assert(!result.abnormal());
    assert(std::abs(result.command_velocity_rad_s - 1.0) < 1e-12);
    assert(std::abs(result.actual_velocity_rad_s - 1.0) < 1e-12);
}

void test_step_and_acceleration_flags()
{
    a10_tcp::JointDiagnosticInput input;
    input.command_position_rad = 0.006;
    input.previous_command_velocity_rad_s = 0.0;
    input.has_previous_command_velocity = true;
    const auto result = a10_tcp::evaluate_joint_diagnostic(input, limits(), k_dt);
    assert(result.flags & a10_tcp::joint_diag_command_step);
    assert(result.flags & a10_tcp::joint_diag_command_acceleration);
    assert(std::abs(result.command_velocity_rad_s - 3.0) < 1e-12);
    assert(std::abs(result.command_acceleration_rad_s2 - 1500.0) < 1e-9);
}

void test_exact_velocity_boundary_is_not_flagged()
{
    a10_tcp::JointDiagnosticInput input;
    input.command_position_rad = 2.0 * k_dt;
    input.previous_command_velocity_rad_s = 2.0;
    input.has_previous_command_velocity = true;
    const auto result = a10_tcp::evaluate_joint_diagnostic(input, limits(), k_dt);
    assert(!result.abnormal());
}

void test_following_error_and_non_finite_flags()
{
    a10_tcp::JointDiagnosticInput following;
    following.command_position_rad = 0.6;
    const auto following_result =
        a10_tcp::evaluate_joint_diagnostic(following, limits(), k_dt);
    assert(following_result.flags & a10_tcp::joint_diag_following_error);

    a10_tcp::JointDiagnosticInput invalid;
    invalid.command_position_rad = std::numeric_limits<double>::quiet_NaN();
    const auto invalid_result =
        a10_tcp::evaluate_joint_diagnostic(invalid, limits(), k_dt);
    assert(invalid_result.flags & a10_tcp::joint_diag_non_finite);
}

void test_actual_velocity_flag_uses_actual_sample_interval()
{
    a10_tcp::JointDiagnosticInput input;
    input.actual_position_rad = 0.012;
    input.previous_actual_position_rad = 0.0;
    input.actual_sample_dt_s = 0.004;
    input.has_previous_actual_position = true;
    const auto result = a10_tcp::evaluate_joint_diagnostic(input, limits(), k_dt);
    assert(result.flags & a10_tcp::joint_diag_actual_velocity);
    assert(std::abs(result.actual_velocity_rad_s - 3.0) < 1e-12);
}
}  // namespace

int main()
{
    test_normal_sample();
    test_step_and_acceleration_flags();
    test_exact_velocity_boundary_is_not_flagged();
    test_following_error_and_non_finite_flags();
    test_actual_velocity_flag_uses_actual_sample_interval();
    std::cout << "a10_vr_joint_diagnostics_test: PASS" << std::endl;
    return 0;
}
