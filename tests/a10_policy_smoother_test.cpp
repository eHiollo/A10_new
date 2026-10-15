#include "a10_policy_smoother.hpp"
#include "a10_policy_batch.hpp"

#include <cassert>
#include <iostream>

void test_smoother(double max_velocity, double max_acceleration = 0.5)
{
    constexpr double dt = 0.002;
    a10_tcp::PolicyCommandSmoother smoother;
    std::vector<double> reference(7, 0.0);
    std::vector<double> previous(7, 0.0);
    std::array<double, 6> previous_velocity{};
    smoother.reset(reference);
    double measured_velocity = 0.0;
    double measured_acceleration = 0.0;
    double max_lag = 0.0;
    for (int tick = 0; tick < 10000; ++tick)
    {
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            if (tick < 600)
                reference[axis] += dt * max_velocity;
            else if (tick < 1200)
                reference[axis] -= dt * max_velocity;
            else if (tick < 2400)
                reference[axis] += (tick / 75 % 2 ? -1.0 : 1.0) * dt * max_velocity;
        }
        reference[6] = tick % 85;
        const auto& command = smoother.step(reference, max_velocity, max_acceleration);
        assert(command[6] == reference[6]);
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            const double velocity = (command[axis] - previous[axis]) / dt;
            const double acceleration = (velocity - previous_velocity[axis]) / dt;
            assert(std::isfinite(command[axis]));
            assert(std::abs(velocity) <= max_velocity + 1e-10);
            assert(std::abs(velocity) <= a10_tcp::VrJointGuard::speed_limit(axis) + 1e-10);
            assert(std::abs(acceleration) <= max_acceleration + 1e-8);
            measured_velocity = std::max(measured_velocity, std::abs(velocity));
            measured_acceleration = std::max(measured_acceleration, std::abs(acceleration));
            previous_velocity[axis] = velocity;
        }
        max_lag = std::max(max_lag, smoother.lag(reference));
        previous = command;
    }
    assert(smoother.settled(reference));
    assert(smoother.lag(reference) < 1e-10);
    smoother.reset(std::vector<double>(7, 0.3));
    assert(smoother.settled(std::vector<double>(7, 0.3)));

    a10_tcp::PolicyBatchQueue queue;
    const auto sequence = queue.replace({reference}, 0.0, a10_tcp::PolicyBatchQueue::Clock::now(), std::nullopt, true);
    assert(queue.status().sequence == sequence && queue.status().smooth);
    queue.replace({reference}, 0.0);
    assert(!queue.status().smooth);
    queue.clear();
    assert(!queue.status().smooth);
    assert(a10_tcp::policy_min_segment_steps(false) == 2);
    const auto minimum_steps = a10_tcp::policy_min_segment_steps(true);
    assert((minimum_steps - 1) * dt >= 1.0 / 30.0);
    assert((minimum_steps - 2) * dt < 1.0 / 30.0);
    assert(a10_tcp::policy_min_segment_steps(false, 0.3) == 2);
    assert(a10_tcp::policy_min_segment_steps(true, 0.3) == 151);
    assert(a10_tcp::policy_min_segment_steps(true, -1.0) == minimum_steps);
    assert(a10_tcp::policy_min_segment_steps(true, 2.0) == 501);
    const int handoff_steps = a10_tcp::policy_min_segment_steps(true, 0.3);
    const double gripper_start = 73.22;
    const double gripper_goal = 69.77;
    const double at_50ms = gripper_start + (gripper_goal - gripper_start) * 25.0 / (handoff_steps - 1);
    assert(at_50ms > 72.6 && at_50ms < gripper_start);
    assert(gripper_start + (gripper_goal - gripper_start) == gripper_goal);
    double duration = 0.0;
    std::vector<double> start(7, 0.0);
    a10_tcp::PolicyBatchQueue::Actions fine_actions;
    for (int index = 0; index < 30; ++index)
    {
        auto goal = start;
        goal[0] += 0.00001;
        goal[6] += 0.01;
        fine_actions.push_back(goal);
        duration += std::max(minimum_steps, a10_tcp::policy_blend_steps(start, goal, max_velocity, 80.0)) * dt;
        start = goal;
    }
    assert(duration >= 1.0 && duration < 1.1);
    const auto now = a10_tcp::PolicyBatchQueue::Clock::now();
    const auto timed_sequence = queue.replace(fine_actions, 0.0, now, std::nullopt, true);
    assert(queue.align(timed_sequence, 1.0 / 30.0, nullptr, now) == 0);
    assert(queue.update_remaining_time(timed_sequence, std::vector<double>(7, 0.0), max_velocity, 80.0));
    assert(std::abs(queue.status().remaining_time_s - duration) < 0.01);
    const auto delayed_sequence = queue.replace(fine_actions, 0.3, now, std::nullopt, true);
    assert(queue.align(delayed_sequence, 1.0 / 30.0, nullptr, now) == 9);
    assert(queue.update_remaining_time(delayed_sequence, std::vector<double>(7, 0.0), max_velocity, 80.0));
    const double delayed_duration = (handoff_steps + 20 * minimum_steps) * dt;
    assert(std::abs(queue.status().remaining_time_s - delayed_duration) < 0.01);
    assert(a10_tcp::policy_blend_steps(std::vector<double>(7, 0.0), std::vector<double>(7, 1.0), max_velocity, 80.0) > minimum_steps);
    std::cout << "smoother tests passed: max_velocity=" << measured_velocity
              << " max_acceleration=" << measured_acceleration << " max_lag=" << max_lag << '\n';
}

void test_guarded_smoother(double working_acceleration)
{
    using Guard = a10_tcp::VrJointGuard;
    Guard guard;
    a10_tcp::PolicyCommandSmoother smoother;
    Guard::Joints previous{}, velocity{}, actual{};
    std::vector<double> command(7, 0.0);
    smoother.reset(command);
    for (int tick = 0; tick < 4000; ++tick)
    {
        std::vector<double> reference(7, 0.0);
        for (std::size_t axis = 0; axis < 6; ++axis)
            reference[axis] = 0.2 * std::sin(tick * 0.003 + axis * 0.1);
        const auto& proposed = smoother.step(reference, Guard::speed_limit(5), working_acceleration);
        Guard::Joints requested{};
        std::copy_n(proposed.begin(), 6, requested.begin());
        const auto result = guard.step(previous, velocity, requested, actual);
        assert(!guard.faulted());
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            assert(std::abs(result.velocity[axis]) <= Guard::speed_limit(axis) + 1e-8);
            assert(std::abs(result.velocity[axis] - velocity[axis]) <= Guard::acceleration * Guard::dt + 1e-8);
            assert(std::abs(result.velocity[axis] - velocity[axis]) <= working_acceleration * Guard::dt + 1e-8);
        }
        previous = result.position;
        velocity = result.velocity;
        actual = previous;
        std::copy(previous.begin(), previous.end(), command.begin());
        smoother.accept(command, velocity);
    }
    actual[0] += 0.2;
    auto result = guard.step(previous, velocity, previous, actual);
    assert(guard.faulted());
    assert(std::string(result.fault) == "joint_following_error");
    for (int tick = 0; tick < 120; ++tick)
    {
        previous = result.position;
        velocity = result.velocity;
        actual = previous;
        result = guard.step(previous, velocity, previous, actual, true);
        for (std::size_t axis = 0; axis < 6; ++axis)
            assert(std::abs(result.velocity[axis]) <= std::abs(velocity[axis]) + 1e-8);
    }
    for (double speed : result.velocity)
        assert(speed == 0.0);
}

int main()
{
    test_guarded_smoother(a10_tcp::VrJointGuard::acceleration);
    test_guarded_smoother(1.0);
    test_smoother(0.05);
    test_smoother(0.075);
    test_smoother(a10_tcp::VrJointGuard::speed_limit(5), a10_tcp::VrJointGuard::acceleration);
    test_smoother(a10_tcp::VrJointGuard::speed_limit(5), 1.0);
}
