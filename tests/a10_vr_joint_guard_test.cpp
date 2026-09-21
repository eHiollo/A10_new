#include "a10_vr_joint_guard.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>

using Guard = a10_tcp::VrJointGuard;
using Joints = Guard::Joints;

static void check(const Guard::Result& r, const Joints& previous, const Joints& velocity)
{
    for (std::size_t i = 0; i < 6; ++i)
    {
        assert(std::isfinite(r.position[i]));
        const double v = (r.position[i] - previous[i]) / Guard::dt;
        assert(std::abs(v) <= Guard::speed_limit(i) + 1e-8);
        assert(std::abs(v - velocity[i]) <= Guard::acceleration * Guard::dt + 1e-8);
        assert(std::abs(r.position[i]) <= Guard::position_limit + 1e-8);
    }
}

static void test_transparent_and_common_scale()
{
    Guard g;
    Joints q{}, v{}, desired{};
    desired[0] = 0.000001;
    desired[3] = -0.000002;
    auto r = g.step(q, v, desired, q);
    assert(!r.limited && !g.faulted());
    assert(r.position == desired);

    // Log cycle 72942: first wrist acceleration surge (+48.99/-37.92).
    desired = {};
    desired[3] = 0.09795051957 * Guard::dt;
    desired[5] = -0.07580514176 * Guard::dt;
    r = g.step(q, v, desired, q);
    check(r, q, v);
    assert(r.limited && !g.faulted() && r.scale < 0.4);
    assert(std::abs(r.position[3] / desired[3] - r.position[5] / desired[5]) < 1e-10);
}

static void test_reversal_and_braking()
{
    Guard g;
    Joints q{}, v{}, target{};
    v[3] = 0.5;
    v[5] = -0.25;
    target[3] = -0.002;
    auto r = g.step(q, v, target, q);
    check(r, q, v);
    assert(r.limited && r.velocity[3] > 0.0); // cannot reverse in 2 ms
    assert(std::abs(r.velocity[5] / r.velocity[3] + 0.5) < 1e-12);
    for (int k = 0; k < 100; ++k)
    {
        q = r.position; v = r.velocity;
        r = g.step(q, v, q, q, true);
        check(r, q, v);
    }
    assert(std::abs(r.velocity[3]) < 1e-12);
}

static void test_faults_and_latching()
{
    Guard g;
    Joints q{}, v{}, target{}, actual{};
    actual[3] = -0.101;
    auto r = g.step(q, v, target, actual);
    assert(g.faulted() && std::strcmp(r.fault, "joint_following_error") == 0 && r.joint == 4);
    target[3] = 0.001;
    for (int k = 0; k < 1000; ++k)
    {
        r = g.step(q, v, target, q); // new targets/valid feedback cannot clear fault
        assert(g.faulted() && r.position == q);
    }
    g.reset();
    target[5] = std::numeric_limits<double>::infinity();
    r = g.step(q, v, target, q);
    check(r, q, v);
    assert(g.faulted() && r.joint == 6);
    g.reset(); target = {}; target[0] = std::numeric_limits<double>::quiet_NaN();
    r = g.step(q, v, target, q);
    check(r, q, v);
    assert(g.faulted() && r.joint == 1);

    g.reset(); target = {}; target[2] = 0.11;
    r = g.step(q, v, target, q);
    assert(g.faulted() && std::strcmp(r.fault, "joint_ik_jump") == 0);

    g.reset(); target = {}; actual = {}; actual[0] = -6.283185307179586;
    r = g.step(q, v, target, actual);
    assert(g.faulted()); // physical motor error must not wrap to zero
}

static void test_sustained_singularity_demand()
{
    Guard g;
    Joints q{}, v{};
    for (unsigned k = 0; k < Guard::limited_cycle_limit + 200; ++k)
    {
        auto target = q;
        target[3] += 6.347445 * Guard::dt;
        target[5] -= 6.847251 * Guard::dt;
        const auto r = g.step(q, v, target, q);
        check(r, q, v);
        q = r.position; v = r.velocity;
    }
    assert(g.faulted());
    assert(std::strcmp(g.fault(), "joint_limit_persistent") == 0);
    for (double speed : v) assert(std::abs(speed) < 1e-12);
}

static void test_position_brake_and_reanchor()
{
    Guard g;
    Joints q{}, v{}, actual_velocity{};
    q[3] = Guard::position_limit - 0.08;
    v[3] = 0.6;
    for (int k = 0; k < 200; ++k)
    {
        auto target = q; target[3] += v[3] * Guard::dt;
        const auto r = g.step(q, v, target, q);
        check(r, q, v);
        q = r.position; v = r.velocity;
    }
    assert(g.faulted() && std::strcmp(g.fault(), "joint_position_limit") == 0);
    assert(Guard::ready_to_anchor(q, v, q, actual_velocity));
    auto actual = q; actual[3] -= 0.02;
    assert(!Guard::ready_to_anchor(q, v, actual, actual_velocity));
    actual_velocity[3] = 0.05;
    assert(!Guard::ready_to_anchor(q, v, q, actual_velocity));

    // A slower axis near a limit must reserve braking distance for coordinated
    // braking with the fastest axis, not assume its own full deceleration.
    g.reset(); q = {}; v = {};
    q[0] = -Guard::position_limit + 0.02; v[0] = -0.15; v[3] = 0.7;
    for (int k = 0; k < 200; ++k)
    {
        auto target = q;
        for (std::size_t i = 0; i < 6; ++i) target[i] += v[i] * Guard::dt;
        const auto r = g.step(q, v, target, q);
        check(r, q, v);
        q = r.position; v = r.velocity;
    }
    assert(g.faulted());
}

static void test_random_requests()
{
    std::mt19937 rng(21);
    std::uniform_real_distribution<double> step(-0.015, 0.015);
    for (int run = 0; run < 100; ++run)
    {
        Guard g;
        Joints q{}, v{};
        for (int k = 0; k < 1000; ++k)
        {
            auto target = q;
            for (auto& angle : target) angle += step(rng);
            const auto r = g.step(q, v, target, q);
            check(r, q, v);
            q = r.position; v = r.velocity;
        }
    }
}

static void test_teleop_envelope_and_ramp()
{
    // Reproduce sustained wrist demand from the Sep 21 trace. Even before the
    // persistent-demand fault, teleop must stay below 45 deg/s and 200 deg/s^2.
    Guard g;
    Joints q{}, v{};
    for (int k = 0; k < 400; ++k)
    {
        auto target = q;
        target[3] -= 3.105133 * Guard::dt;
        target[5] += 2.589455389 * Guard::dt;
        const auto r = g.step(q, v, target, q);
        for (std::size_t i = 0; i < 6; ++i)
        {
            assert(std::abs(r.velocity[i]) <= 0.785398164);
            assert(std::abs(r.velocity[i] - v[i]) / Guard::dt <= 3.490658505);
        }
        check(r, q, v);
        q = r.position; v = r.velocity;
    }
    assert(g.faulted()); // sustained excessive demand still locks out

    // A feasible request needs >100 ms to ramp with the gentler acceleration.
    g.reset(); q = {}; v = {};
    for (int k = 0; k < 200; ++k)
    {
        auto target = q; target[3] += 0.6 * Guard::dt;
        const auto r = g.step(q, v, target, q);
        check(r, q, v);
        assert(!g.faulted());
        q = r.position; v = r.velocity;
    }
    assert(std::abs(v[3] - 0.6) < 1e-10);
}

static void test_release_brakes_without_persistent_fault()
{
    Guard g;
    Joints q{}, v{};
    v[3] = -Guard::speed_limit(3);
    v[5] = 0.8 * Guard::speed_limit(5);
    // A release near singularity must brake for longer than the old 50-cycle
    // timeout without becoming a fault. Feedback is ideal here, not a servo model.
    unsigned braking_cycles = 0;
    for (int k = 0; k < 400; ++k)
    {
        const auto r = g.step(q, v, q, q, true);
        check(r, q, v);
        assert(!g.faulted());
        assert(std::abs(r.velocity[3]) <= std::abs(v[3]));
        if (std::abs(r.velocity[3]) > 1e-12) ++braking_cycles;
        q = r.position; v = r.velocity;
    }
    assert(braking_cycles > 50);
    for (double speed : v) assert(std::abs(speed) < 1e-12);
    assert(Guard::ready_to_anchor(q, v, q, Joints{}));
    auto target = q; target[3] += 0.000001;
    const auto r = g.step(q, v, target, q);
    assert(!g.faulted() && r.position[3] > q[3]);

    // Release must still latch a genuine following error.
    auto actual = q; actual[3] += 0.11;
    g.step(q, v, q, actual, true);
    assert(g.faulted() && std::strcmp(g.fault(), "joint_following_error") == 0);
}

int main()
{
    {
        Guard g;
        Joints minimum{}, maximum{}, q{}, v{};
        minimum.fill(-Guard::position_limit); maximum.fill(Guard::position_limit);
        minimum[3] = -3.14159265358979323846;
        assert(g.set_position_limits(minimum, maximum));
        // J4 can move below the old -175 deg bound while other limits stay intact.
        q[3] = -3.09;
        auto target = q; target[3] -= 0.000001;
        auto r = g.step(q, v, target, q);
        assert(!g.faulted() && r.position[3] < q[3]);
        g.reset(); q = {}; q[2] = -3.09;
        r = g.step(q, v, q, q);
        assert(g.faulted() && r.joint == 3);
        g.reset(); q = {}; q[3] = 3.09;
        r = g.step(q, v, q, q);
        assert(g.faulted() && r.joint == 4);
        // Braking must use the new lower bound, still reserving stop distance.
        g.reset(); q = {}; q[3] = minimum[3] + 0.006; v[3] = -0.15;
        for (int k = 0; k < 100; ++k)
        {
            target = q; target[3] += v[3] * Guard::dt;
            r = g.step(q, v, target, q);
            assert(r.position[3] >= minimum[3]);
            assert(std::abs(r.velocity[3] - v[3]) <= Guard::acceleration * Guard::dt + 1e-10);
            q = r.position; v = r.velocity;
        }
        assert(g.faulted() && std::abs(v[3]) < 1e-12);
        minimum[3] = std::numeric_limits<double>::quiet_NaN();
        assert(!g.set_position_limits(minimum, maximum));
        assert(g.minimum(3) == -3.14159265358979323846); // failed config is atomic
    }
    test_teleop_envelope_and_ramp();
    test_release_brakes_without_persistent_fault();
    test_transparent_and_common_scale();
    test_reversal_and_braking();
    test_faults_and_latching();
    test_sustained_singularity_demand();
    test_position_brake_and_reanchor();
    test_random_requests();
    std::cout << "a10_vr_joint_guard_test: 9 groups PASS\n";
}
