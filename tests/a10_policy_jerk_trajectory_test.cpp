#include "a10_policy_jerk_trajectory.hpp"
#include "a10_policy_batch.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>

using a10_tcp::PolicyJerkTrajectory;
using a10_tcp::VrJointGuard;

struct Motion
{
    std::vector<double> position = std::vector<double>(7, 0.0);
    VrJointGuard::Joints velocity{}, acceleration{};
    VrJointGuard guard;
    double peak_jerk{0.0};

    void step(const std::vector<double>& requested)
    {
        VrJointGuard::Joints previous{}, target{};
        std::copy_n(position.begin(), 6, previous.begin());
        std::copy_n(requested.begin(), 6, target.begin());
        const auto accepted = guard.step(previous, velocity, target, previous);
        assert(!guard.faulted() && !accepted.limited);
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            const double next_acceleration = (accepted.velocity[axis] - velocity[axis]) / PolicyJerkTrajectory::dt;
            const double jerk = (next_acceleration - acceleration[axis]) / PolicyJerkTrajectory::dt;
            peak_jerk = std::max(peak_jerk, std::abs(jerk));
            assert(std::abs(accepted.velocity[axis]) <= VrJointGuard::speed_limit(axis) + 1e-8);
            assert(std::abs(next_acceleration) <= 1.0 + 1e-7);
            assert(std::abs(jerk) <= 20.0 + 1e-5);
            acceleration[axis] = next_acceleration;
        }
        assert(std::abs(requested[6] - position[6]) <= 50.0 * PolicyJerkTrajectory::dt + 1e-8);
        position = requested;
        velocity = accepted.velocity;
    }

    void follow(const PolicyJerkTrajectory& trajectory, int ticks)
    {
        assert(trajectory.sample(0) == position);
        for (int history = 1; history <= 2; ++history)
        {
            const auto past = trajectory.sample(-history);
            for (std::size_t axis = 0; axis < 6; ++axis)
            {
                const double expected = position[axis] - history * velocity[axis] * PolicyJerkTrajectory::dt
                    + (history == 2 ? acceleration[axis] * PolicyJerkTrajectory::dt * PolicyJerkTrajectory::dt : 0.0);
                assert(std::abs(past[axis] - expected) < 1e-12);
            }
        }
        for (int tick = 1; tick <= ticks; ++tick)
            step(trajectory.sample(tick));
    }
};

void test_retarget_brake_and_hold()
{
    Motion motion;
    std::size_t rejected = 0;
    double max_plan_us = 0.0;
    PolicyJerkTrajectory trajectory;
    for (int segment = 0; segment < 240; ++segment)
    {
        auto goal = motion.position;
        for (std::size_t axis = 0; axis < 6; ++axis)
            goal[axis] = 0.25 * std::sin(segment * 0.37 + axis * 0.4);
        goal[6] = segment % 7 < 3 ? 75.0 : 2.0;
        const auto begin = std::chrono::steady_clock::now();
        if (!trajectory.plan(motion.position, motion.velocity, motion.acceleration, goal, {},
            1.0 / 30.0, 0.785, 1.0, 20.0, 50.0))
        {
            ++rejected;
            assert(trajectory.brake(motion.position, motion.velocity, motion.acceleration, 0.785, 1.0, 20.0, 50.0));
        }
        max_plan_us = std::max(max_plan_us, std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - begin).count());
        motion.follow(trajectory, std::min(trajectory.ticks(), 135 + segment % 37));
    }
    assert(trajectory.brake(motion.position, motion.velocity, motion.acceleration, 0.785, 1.0, 20.0, 50.0));
    motion.follow(trajectory, trajectory.ticks());
    for (int tick = 0; tick < 4; ++tick)
        motion.step(motion.position);
    assert(rejected < 24);
    std::cout << "retargets=240 rejected=" << rejected << " max_plan_us=" << max_plan_us
              << " max_command_jerk=" << motion.peak_jerk << '\n';
}

void test_nonzero_endpoint_and_brake()
{
    Motion motion;
    PolicyJerkTrajectory trajectory;
    auto goal = motion.position;
    goal[0] = 0.2;
    VrJointGuard::Joints end_velocity{};
    end_velocity[0] = 0.2;
    assert(trajectory.plan(motion.position, motion.velocity, motion.acceleration, goal, end_velocity,
        0.034, 0.785, 1.0, 20.0, 50.0));
    motion.follow(trajectory, trajectory.ticks());
    assert(motion.velocity[0] > 0.19);
    assert(trajectory.brake(motion.position, motion.velocity, motion.acceleration, 0.785, 1.0, 20.0, 50.0));
    motion.follow(trajectory, trajectory.ticks() / 2);
    goal[0] = -0.1;
    assert(trajectory.plan(motion.position, motion.velocity, motion.acceleration, goal, {},
        0.034, 0.785, 1.0, 20.0, 50.0));
    motion.follow(trajectory, trajectory.ticks());
    for (int tick = 0; tick < 4; ++tick)
        motion.step(motion.position);
    assert(motion.position == goal);
}

void test_validation()
{
    PolicyJerkTrajectory trajectory;
    const std::vector<double> zero(7, 0.0);
    auto goal = zero;
    goal[0] = 0.2;
    assert(!trajectory.configure(zero, {}, {}, goal, {}, 0.1, 0.785, 1.0, 20.0, 50.0));
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()})
        assert(!trajectory.configure(zero, {}, {}, goal, {}, 1.0, 0.785, 1.0, invalid, 50.0));
    assert(!trajectory.plan(zero, {}, {}, goal, {}, -1.0, 0.785, 1.0, 20.0, 50.0));
    assert(!trajectory.configure({}, {}, {}, goal, {}, 1.0, 0.785, 1.0, 20.0, 50.0));
    auto acceleration = VrJointGuard::Joints{};
    acceleration[0] = 2.0;
    assert(!trajectory.plan(zero, {}, acceleration, goal, {}, 0.034, 0.785, 1.0, 20.0, 50.0));
    goal[0] = std::numeric_limits<double>::quiet_NaN();
    assert(!trajectory.plan(zero, {}, {}, goal, {}, 0.034, 0.785, 1.0, 20.0, 50.0));
}

void test_streaming_and_queue_generations()
{
    using Queue = a10_tcp::PolicyBatchQueue;
    Queue queue;
    Motion motion;
    const auto now = Queue::Clock::now();
    std::size_t fallbacks = 0;
    std::size_t rejected = 0;
    double max_selection_us = 0.0;
    PolicyJerkTrajectory active;
    int active_tick = 0;
    std::uint64_t active_sequence = 0;
    for (int batch = 0; batch < 160; ++batch)
    {
        Queue::Actions actions;
        const double age = 0.3 + 0.08 * std::abs(std::sin(batch));
        for (int index = 0; index < 30; ++index)
        {
            std::vector<double> goal(7, 0.0);
            for (std::size_t axis = 0; axis < 6; ++axis)
                goal[axis] = 0.12 * std::sin((batch * 0.3 - age + index / 30.0) * 0.8 + axis * 0.2);
            goal[6] = 15.0 + 10.0 * std::sin((batch * 0.3 - age + index / 30.0) * 0.8);
            actions.push_back(goal);
        }
        const auto sequence = queue.replace(actions, age, now, std::nullopt, true);
        assert(!queue.status().jerk_limited && !queue.status().plan_rejected);
        queue.align(sequence, 1.0 / 30.0, nullptr, now);
        const auto begin = std::chrono::steady_clock::now();
        const bool selected = queue.select_handoff(sequence, 1.0 / 30.0, motion.position, 0.785, 50.0, 0.5, 85.0,
            motion.velocity, 1.0, motion.acceleration, 20.0);
        max_selection_us = std::max(max_selection_us, std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - begin).count());
        const auto status = queue.status();
        if (selected)
        {
            assert(status.jerk_limited && status.handoff_selected);
            fallbacks += status.handoff_fallback;
        }
        else
        {
            ++rejected;
            assert(queue.reject(sequence));
        }
        double minimum = status.handoff_duration_s;
        bool retarget = selected;
        for (int tick = 0; tick < 150; ++tick)
        {
            if (retarget || active_tick >= active.ticks())
            {
                retarget = false;
                std::vector<double> goal;
                if (queue.peek(goal, sequence))
                {
                    VrJointGuard::Joints endpoint{};
                    assert(queue.endpoint_velocity(sequence, 1.0 / 30.0, 0.785, endpoint));
                    PolicyJerkTrajectory next;
                    if (next.configure(motion.position, motion.velocity, motion.acceleration, goal, endpoint,
                        minimum, 0.785, 1.0, 20.0, 50.0) ||
                        next.plan(motion.position, motion.velocity, motion.acceleration, goal, {},
                            minimum, 0.785, 1.0, 20.0, 50.0))
                    {
                        active = next;
                        active_tick = 0;
                        active_sequence = sequence;
                    }
                    else
                    {
                        ++rejected;
                        assert(queue.reject(sequence));
                    }
                    minimum = 1.0 / 30.0;
                }
                else if (active_tick >= active.ticks() && std::any_of(active.end_velocity().begin(),
                    active.end_velocity().end(), [](double value) { return value != 0.0; }))
                {
                    assert(active.brake(motion.position, motion.velocity, motion.acceleration, 0.785, 1.0, 20.0, 50.0));
                    active_tick = 0;
                    active_sequence = sequence;
                }
            }
            if (active_tick < active.ticks())
            {
                motion.step(active.sample(++active_tick));
                if (active_tick == active.ticks())
                    queue.pop(active_sequence);
            }
            else
                motion.step(motion.position);
        }
        assert(!queue.reject(sequence - 1));
        assert(queue.reject(sequence));
        assert(queue.status().remaining == 0 && queue.status().plan_rejected);
        assert(queue.status().sequence == sequence);
    }
    const auto sequence = queue.replace(Queue::Actions(30, motion.position), 1.1, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    assert(queue.status().expired && !queue.status().plan_rejected);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, motion.position, 0.785, 50.0, 0.5, 85.0,
        motion.velocity, 1.0, motion.acceleration, 20.0));
    assert(!queue.status().handoff_selected);
    assert(rejected < 16);
    std::cout << "streaming fallbacks=" << fallbacks << "/160 rejected=" << rejected << " max_selection_us=" << max_selection_us
              << " max_command_jerk=" << motion.peak_jerk << '\n';
}

int main()
{
    test_validation();
    test_nonzero_endpoint_and_brake();
    test_retarget_brake_and_hold();
    test_streaming_and_queue_generations();
    std::cout << "policy jerk trajectory tests passed\n";
}
