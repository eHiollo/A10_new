#include "a10_anchor_protocol.hpp"
#include "a10_vr_reset_command.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
constexpr double k_pi = 3.14159265358979323846;

void expect_near(double actual, double expected)
{
    assert(std::abs(actual - expected) < 1e-10);
}

a10_tcp::EeAnchorCommand command(
    std::uint64_t anchor_id, std::uint64_t sample_sequence, double x)
{
    a10_tcp::EeAnchorCommand value;
    value.active = true;
    value.session_id = "session-a";
    value.anchor_id = anchor_id;
    value.sample_sequence = sample_sequence;
    value.offset = {x, 0.0, 0.0, 0.0, 0.0, 0.0};
    return value;
}

void test_protocol_parser()
{
    a10_tcp::EeAnchorCommand parsed;
    std::string error;
    const std::string valid =
        "SET_EE_ANCHOR {\"active\":true,\"session_id\":\"abc\",\"anchor_id\":2,"
        "\"sample_sequence\":9,\"offset\":[0.1,0.2,0.3,0.0,0.0,0.4],"
        "\"gripper\":-0.5}";
    assert(a10_tcp::parse_ee_anchor_line(valid, parsed, &error));
    assert(error.empty());
    assert(parsed.active);
    assert(parsed.session_id == "abc");
    assert(parsed.anchor_id == 2);
    assert(parsed.sample_sequence == 9);
    expect_near(parsed.offset[2], 0.3);
    expect_near(parsed.gripper, -0.5);

    const std::string typed =
        "{\"type\":\"SET_EE_ANCHOR\",\"active\":false,\"session_id\":\"abc\","
        "\"anchor_id\":2,\"sample_sequence\":10,\"offset\":[0,0,0,0,0,0],"
        "\"gripper\":0}";
    assert(a10_tcp::parse_ee_anchor_line(typed, parsed, &error));
    assert(!parsed.active);

    assert(!a10_tcp::parse_ee_anchor_line(
        "SET_EE_ANCHOR {\"active\":true,\"session_id\":\"abc\",\"anchor_id\":2,"
        "\"sample_sequence\":9,\"offset\":[0,0,0],\"gripper\":0}",
        parsed,
        &error));
    assert(!error.empty());
    assert(!a10_tcp::parse_ee_anchor_line(
        "SET_EE_ANCHOR {\"active\":true,\"session_id\":\"abc\",\"anchor_id\":-1,"
        "\"sample_sequence\":9,\"offset\":[0,0,0,0,0,0],\"gripper\":0}",
        parsed,
        &error));
    assert(!a10_tcp::parse_ee_anchor_line(
        "SET_EE_ANCHOR_BAD {\"active\":true,\"session_id\":\"abc\",\"anchor_id\":2,"
        "\"sample_sequence\":9,\"offset\":[0,0,0,0,0,0],\"gripper\":0}",
        parsed,
        &error));
}

void test_shadow_state_and_transform()
{
    const double identity[16] = {
        1.0, 0.0, 0.0, 1.0,
        0.0, 1.0, 0.0, 2.0,
        0.0, 0.0, 1.0, 3.0,
        0.0, 0.0, 0.0, 1.0,
    };
    a10_tcp::EeAnchorShadow shadow;
    auto first = command(1, 10, 0.1);
    first.offset[5] = k_pi / 2.0;
    assert(shadow.update(first, identity) == a10_tcp::AnchorShadowUpdate::anchored);
    expect_near(shadow.robot_anchor_pm()[3], 1.0);
    expect_near(shadow.user_target_pm()[3], 1.1);
    expect_near(shadow.user_target_pm()[0], 0.0);
    expect_near(shadow.user_target_pm()[1], -1.0);
    expect_near(shadow.user_target_pm()[4], 1.0);

    const double moved_actual[16] = {
        1.0, 0.0, 0.0, 9.0,
        0.0, 1.0, 0.0, 8.0,
        0.0, 0.0, 1.0, 7.0,
        0.0, 0.0, 0.0, 1.0,
    };
    auto update = command(1, 11, 0.2);
    assert(shadow.update(update, moved_actual) == a10_tcp::AnchorShadowUpdate::updated);
    expect_near(shadow.robot_anchor_pm()[3], 1.0);
    expect_near(shadow.user_target_pm()[3], 1.2);

    auto duplicate = command(1, 11, 5.0);
    assert(shadow.update(duplicate, moved_actual) == a10_tcp::AnchorShadowUpdate::duplicate);
    expect_near(shadow.user_target_pm()[3], 1.2);

    auto stale_sample = command(1, 9, 6.0);
    assert(shadow.update(stale_sample, moved_actual) == a10_tcp::AnchorShadowUpdate::stale);
    expect_near(shadow.user_target_pm()[3], 1.2);

    auto reanchor = command(2, 12, 0.0);
    assert(shadow.update(reanchor, moved_actual) == a10_tcp::AnchorShadowUpdate::anchored);
    expect_near(shadow.robot_anchor_pm()[3], 9.0);
    expect_near(shadow.user_target_pm()[3], 9.0);

    auto stale_anchor = command(1, 13, 1.0);
    assert(shadow.update(stale_anchor, moved_actual) == a10_tcp::AnchorShadowUpdate::stale);
    expect_near(shadow.user_target_pm()[3], 9.0);

    const double rotated_actual[16] = {
        0.0, -1.0, 0.0, 4.0,
        1.0, 0.0, 0.0, 5.0,
        0.0, 0.0, 1.0, 6.0,
        0.0, 0.0, 0.0, 1.0,
    };
    auto new_session = command(1, 1, 0.25);
    new_session.session_id = "session-b";
    assert(
        shadow.update(new_session, rotated_actual)
        == a10_tcp::AnchorShadowUpdate::anchored);
    expect_near(shadow.robot_anchor_pm()[3], 4.0);
    expect_near(shadow.user_target_pm()[3], 4.0);
    expect_near(shadow.user_target_pm()[7], 5.25);

    auto release = command(0, 0, 0.0);
    release.session_id = "restarted-session";
    release.active = false;
    assert(shadow.update(release, moved_actual) == a10_tcp::AnchorShadowUpdate::inactive);
    assert(!shadow.active());
    expect_near(shadow.user_target_pm()[7], 5.25);
}

void test_reference_governor_limits_translation_and_rotation()
{
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    const double user_target[16] = {
        0.0, -1.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    a10_tcp::AnchorGovernorConfig config;
    config.max_reference_linear_speed_m_s = 0.1;
    config.max_reference_angular_speed_rad_s = 0.2;
    config.max_tracking_error_m = 2.0;
    config.max_tracking_error_rad = k_pi;
    a10_tcp::EeAnchorReferenceGovernor governor;
    governor.set_config(config);
    governor.engage(identity);

    assert(governor.step(user_target, identity, 0.1) == a10_tcp::AnchorControlState::tracking);
    const auto& reference = governor.reference_pm();
    expect_near(reference[3], 0.01);
    expect_near(reference[7], 0.0);
    expect_near(reference[11], 0.0);
    expect_near(reference[0], std::cos(0.02));
    expect_near(reference[1], -std::sin(0.02));
    expect_near(reference[4], std::sin(0.02));
    expect_near(reference[5], std::cos(0.02));
}

void test_reference_governor_freezes_faults_and_discards_backlog()
{
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    double user_target[16];
    std::copy_n(identity, 16, user_target);
    user_target[3] = 1.0;

    a10_tcp::AnchorGovernorConfig config;
    config.max_reference_linear_speed_m_s = 1.0;
    config.max_reference_angular_speed_rad_s = 1.0;
    config.max_tracking_error_m = 0.05;
    config.max_tracking_error_rad = 0.2;
    config.fault_after_frozen_cycles = 2;
    a10_tcp::EeAnchorReferenceGovernor governor;
    governor.set_config(config);
    governor.engage(identity);

    assert(governor.step(user_target, identity, 0.1) == a10_tcp::AnchorControlState::tracking);
    expect_near(governor.reference_pm()[3], 0.1);
    assert(governor.step(user_target, identity, 0.1) == a10_tcp::AnchorControlState::frozen);
    expect_near(governor.reference_pm()[3], 0.1);
    assert(governor.frozen_cycles() == 1);
    assert(governor.step(user_target, identity, 0.1) == a10_tcp::AnchorControlState::fault);
    expect_near(governor.reference_pm()[3], 0.0);
    assert(!governor.control_active());

    double reanchor[16];
    std::copy_n(identity, 16, reanchor);
    reanchor[3] = 0.4;
    governor.engage(reanchor);
    assert(governor.state() == a10_tcp::AnchorControlState::tracking);
    expect_near(governor.reference_pm()[3], 0.4);
    governor.release(reanchor);
    assert(governor.state() == a10_tcp::AnchorControlState::inactive);
    expect_near(governor.reference_pm()[3], 0.4);
}

void test_reference_governor_recovers_from_short_freeze()
{
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    double user_target[16];
    std::copy_n(identity, 16, user_target);
    user_target[3] = 0.5;

    a10_tcp::AnchorGovernorConfig config;
    config.max_reference_linear_speed_m_s = 1.0;
    config.max_tracking_error_m = 0.05;
    config.fault_after_frozen_cycles = 3;
    a10_tcp::EeAnchorReferenceGovernor governor;
    governor.set_config(config);
    governor.engage(identity);
    governor.step(user_target, identity, 0.1);

    assert(governor.step(user_target, identity, 0.1) == a10_tcp::AnchorControlState::frozen);
    double caught_up[16];
    std::copy_n(identity, 16, caught_up);
    caught_up[3] = 0.1;
    assert(governor.step(user_target, caught_up, 0.1) == a10_tcp::AnchorControlState::tracking);
    expect_near(governor.reference_pm()[3], 0.2);
    assert(governor.frozen_cycles() == 0);
}

void test_reference_governor_handles_pi_rotation_with_mixed_axis_signs()
{
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    const double pi_target[16] = {
        0.0, -1.0, 0.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    a10_tcp::AnchorGovernorConfig config;
    config.max_reference_angular_speed_rad_s = 0.1;
    config.max_tracking_error_m = 1.0;
    config.max_tracking_error_rad = k_pi;
    a10_tcp::EeAnchorReferenceGovernor governor;
    governor.set_config(config);
    governor.engage(identity);
    governor.step(pi_target, identity, 1.0);

    assert(governor.reference_pm()[2] < 0.0);
    assert(governor.reference_pm()[6] < 0.0);
}

void test_anchor_speed_cap_and_smooth_stop_step()
{
    expect_near(a10_tcp::effective_anchor_speed_limit(0.12, 0.06), 0.06);
    expect_near(a10_tcp::effective_anchor_speed_limit(0.04, 0.06), 0.04);

    constexpr double dt_s = 0.002;
    constexpr double max_acc_m_s2 = 0.8;
    const double first_stop_step =
        a10_tcp::slew_toward(0.06, 0.0, max_acc_m_s2 * dt_s);
    expect_near(first_stop_step, 0.0584);
    assert(first_stop_step > 0.0);

    double linear_velocity = 0.06;
    int linear_steps = 0;
    while (linear_velocity > 0.0)
    {
        const double previous = linear_velocity;
        linear_velocity = a10_tcp::slew_toward(
            linear_velocity, 0.0, max_acc_m_s2 * dt_s);
        assert(linear_velocity >= 0.0);
        assert(linear_velocity < previous);
        ++linear_steps;
    }
    assert(linear_steps == 38);

    double angular_velocity = 0.2;
    int angular_steps = 0;
    while (angular_velocity > 0.0)
    {
        angular_velocity = a10_tcp::slew_toward(
            angular_velocity, 0.0, 1.5 * dt_s);
        ++angular_steps;
    }
    assert(angular_steps == 67);
}

void test_joint_limit_hold_keeps_tracking_monitor_live()
{
    const double identity[16] = {
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    double target[16], actual[16];
    std::copy_n(identity, 16, target);
    std::copy_n(identity, 16, actual);
    target[3] = 0.1;
    a10_tcp::EeAnchorReferenceGovernor governor;
    a10_tcp::AnchorGovernorConfig config;
    config.fault_after_frozen_cycles = 2;
    governor.set_config(config);
    governor.engage(identity);
    assert(governor.step(target, actual, 0.002, true) == a10_tcp::AnchorControlState::frozen);
    expect_near(governor.reference_pm()[3], 0.0);
    actual[3] = 0.02;
    governor.step(target, actual, 0.002, true);
    expect_near(governor.tracking_error_m(), 0.02);
    expect_near(governor.reference_pm()[3], 0.0);
    assert(governor.step(target, actual, 0.002) == a10_tcp::AnchorControlState::tracking);
    assert(governor.reference_pm()[3] > 0.0);
    actual[3] = -0.1;
    assert(governor.step(target, actual, 0.002, true) == a10_tcp::AnchorControlState::frozen);
    assert(governor.step(target, actual, 0.002, true) == a10_tcp::AnchorControlState::fault);
}
}  // namespace

int main()
{
    // Optional timing metadata must not silently accept invalid timestamps.
    {
        a10_tcp::EeAnchorCommand parsed;
        std::string error;
        assert(!a10_tcp::parse_ee_anchor_line(
            "SET_EE_ANCHOR {\"active\":true,\"session_id\":\"timing\","
            "\"anchor_id\":1,\"sample_sequence\":1,\"offset\":[0,0,0,0,0,0],"
            "\"gripper\":0,\"client_sample_time_ns\":-1}", parsed, &error));
        const std::string base = "SET_EE_ANCHOR {\"active\":true,\"session_id\":\"timing\","
            "\"anchor_id\":1,\"sample_sequence\":1,\"offset\":[0,0,0,0.1,-0.2,0.3],\"gripper\":0";
        assert(a10_tcp::parse_ee_anchor_line(base + ",\"client_sample_time_ns\":1234567890123456,"
            "\"client_send_time_ns\":1234567891123456,\"robot_receive_time_ns\":99}", parsed, &error));
        assert(parsed.has_client_sample_time && parsed.has_client_send_time);
        assert(parsed.client_sample_time_ns == 1234567890123456ULL);
        assert(parsed.client_send_time_ns == 1234567891123456ULL);
        assert(parsed.robot_receive_time_ns == 0 && parsed.robot_publish_time_ns == 0);
        expect_near(parsed.offset[4], -0.2);
        assert(!a10_tcp::parse_ee_anchor_line(base + ",\"client_sample_time_ns\":2,\"client_send_time_ns\":1}", parsed, &error));
        assert(!a10_tcp::parse_ee_anchor_line(base + ",\"client_send_time_ns\":1.5}", parsed, &error));
        assert(!a10_tcp::parse_ee_anchor_line(base + ",\"client_send_time_ns\":\"123\"}", parsed, &error));
        assert(a10_tcp::parse_ee_anchor_line(base + ",\"client_send_time_ns\":0}", parsed, &error));
        assert(!parsed.has_client_sample_time && parsed.has_client_send_time);
        // Reusing output for a legacy packet clears metadata from the last packet.
        assert(a10_tcp::parse_ee_anchor_line(base + "}", parsed, &error));
        assert(!parsed.has_client_sample_time && !parsed.has_client_send_time);
        assert(parsed.client_sample_time_ns == 0 && parsed.client_send_time_ns == 0);
    }
    assert(a10_tcp::is_vr_reset_command("reset"));
    assert(a10_tcp::is_vr_reset_command("RESET"));
    assert(a10_tcp::is_vr_reset_command("  Reset\r"));
    assert(!a10_tcp::is_vr_reset_command("reset now"));
    assert(!a10_tcp::is_vr_reset_command("SET_EE_DELTA {\"actions\":[0,0,0,0,0,0,0]}"));
    test_protocol_parser();
    test_shadow_state_and_transform();
    test_reference_governor_limits_translation_and_rotation();
    test_reference_governor_freezes_faults_and_discards_backlog();
    test_reference_governor_recovers_from_short_freeze();
    test_reference_governor_handles_pi_rotation_with_mixed_axis_signs();
    test_anchor_speed_cap_and_smooth_stop_step();
    test_joint_limit_hold_keeps_tracking_monitor_live();
    std::cout << "a10_anchor_protocol_test: PASS" << std::endl;
    return 0;
}
