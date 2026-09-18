#include "a10_anchor_protocol.hpp"

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
}  // namespace

int main()
{
    test_protocol_parser();
    test_shadow_state_and_transform();
    std::cout << "a10_anchor_protocol_test: PASS" << std::endl;
    return 0;
}
