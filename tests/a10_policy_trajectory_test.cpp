#include "a10_policy_trajectory.hpp"
#include "a10_policy_batch.hpp"

#include <cassert>
#include <iostream>
#include <limits>

using a10_tcp::PolicyTrajectory;
using a10_tcp::VrJointGuard;

void check_curve(const PolicyTrajectory& trajectory, std::vector<double>& position,
                 VrJointGuard::Joints& velocity, VrJointGuard& guard, int ticks, double acceleration)
{
    const auto initial_position = position;
    for (int tick = 1; tick <= ticks; ++tick)
    {
        const auto requested = trajectory.sample(tick);
        VrJointGuard::Joints previous{}, target{};
        std::copy_n(position.begin(), 6, previous.begin());
        std::copy_n(requested.begin(), 6, target.begin());
        const auto accepted = guard.step(previous, velocity, target, previous);
        assert(!guard.faulted() && !accepted.limited);
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            assert(std::abs(accepted.velocity[axis]) <= VrJointGuard::speed_limit(axis) + 1e-8);
            assert(std::abs(accepted.velocity[axis] - velocity[axis]) <= acceleration * PolicyTrajectory::dt + 1e-8);
            assert(std::abs(accepted.position[axis] - requested[axis]) < 1e-9);
        }
        assert(std::abs(requested[6] - position[6]) <= 50.0 * PolicyTrajectory::dt + 1e-8);
        position = requested;
        velocity = accepted.velocity;
    }
    assert(trajectory.sample(0) == initial_position);
}

void test_retargets_and_settle()
{
    std::vector<double> position(7, 0.0);
    VrJointGuard::Joints velocity{};
    VrJointGuard guard;
    PolicyTrajectory trajectory;
    for (int segment = 0; segment < 160; ++segment)
    {
        auto goal = position;
        for (std::size_t axis = 0; axis < 6; ++axis)
            goal[axis] = 0.25 * std::sin(segment * 0.37 + axis * 0.4);
        goal[6] = segment % 7 < 3 ? 75.0 : 2.0;
        assert(trajectory.plan(position, velocity, goal, {}, 1.0 / 30.0,
            VrJointGuard::speed_limit(5), 1.0, 50.0));
        const int ticks = std::min(trajectory.ticks(), 135 + segment % 37);
        check_curve(trajectory, position, velocity, guard, ticks, 1.0);
    }
    const std::vector<double> goal(7, 0.0);
    assert(trajectory.plan(position, velocity, goal, {}, 1.0 / 30.0, VrJointGuard::speed_limit(5), 1.0, 50.0));
    check_curve(trajectory, position, velocity, guard, trajectory.ticks(), 1.0);
    assert(position == goal);
    VrJointGuard::Joints final_position{};
    const auto stopped = guard.step(final_position, velocity, final_position, final_position);
    assert(!guard.faulted() && !stopped.limited);
    for (std::size_t axis = 0; axis < 6; ++axis)
        assert(std::abs(stopped.velocity[axis] - velocity[axis]) <= PolicyTrajectory::dt + 1e-8);
}

void test_streaming_aligned_chunks()
{
    using Queue = a10_tcp::PolicyBatchQueue;
    const auto now = Queue::Clock::now();
    Queue queue;
    std::vector<double> position(7, 0.0);
    VrJointGuard::Joints velocity{};
    VrJointGuard guard;
    double time = 0.0;
    std::size_t fallbacks = 0;
    for (int batch = 0; batch < 160; ++batch)
    {
        Queue::Actions actions;
        const double age = 0.3 + 0.08 * std::abs(std::sin(batch));
        for (int index = 0; index < 30; ++index)
        {
            std::vector<double> goal(7, 0.0);
            for (std::size_t axis = 0; axis < 6; ++axis)
                goal[axis] = 0.12 * std::sin((time - age + index / 30.0) * 0.8 + axis * 0.2);
            goal[6] = 15.0 + 10.0 * std::sin((time - age + index / 30.0) * 0.8);
            actions.push_back(goal);
        }
        const auto sequence = queue.replace(actions, age, now, std::nullopt, true);
        queue.align(sequence, 1.0 / 30.0, nullptr, now);
        assert(queue.select_handoff(sequence, 1.0 / 30.0, position, 0.785, 50, 0.5, 85, velocity, 1.0));
        const auto status = queue.status();
        fallbacks += status.handoff_fallback;
        if (!status.handoff_fallback)
            assert(status.handoff_late_s <= PolicyTrajectory::dt + 1e-8);
        double minimum = status.handoff_duration_s;
        int remaining_ticks = 150;
        std::vector<double> goal;
        while (remaining_ticks > 0 && queue.peek(goal, sequence))
        {
            VrJointGuard::Joints endpoint{};
            assert(queue.endpoint_velocity(sequence, 1.0 / 30.0, 0.785, endpoint));
            PolicyTrajectory trajectory;
            if (!trajectory.configure(position, velocity, goal, endpoint, minimum, 0.785, 1.0, 50))
                assert(trajectory.plan(position, velocity, goal, {}, minimum, 0.785, 1.0, 50));
            const int ticks = std::min(remaining_ticks, trajectory.ticks());
            check_curve(trajectory, position, velocity, guard, ticks, 1.0);
            remaining_ticks -= ticks;
            if (ticks == trajectory.ticks())
                assert(queue.pop(sequence));
            minimum = 1.0 / 30.0;
        }
        time += 0.3;
    }
    assert(fallbacks < 5);
    std::cout << "streaming fallback batches: " << fallbacks << "/160\n";
}

void test_invalid_and_infeasible()
{
    const std::vector<double> zero(7, 0.0);
    auto goal = zero;
    goal[0] = 0.2;
    PolicyTrajectory trajectory;
    assert(!trajectory.configure(zero, {}, goal, {}, 0.3, 0.785, 1.0, 50));
    assert(trajectory.plan(zero, {}, goal, {}, 0.3, 0.785, 1.0, 50));
    assert(trajectory.duration() > 1.0);
    assert(trajectory.sample(trajectory.ticks()) == goal);
    assert(!trajectory.configure(zero, {}, goal, {}, -1.0, 0.785, 1.0, 50));
    assert(!trajectory.configure(zero, {}, goal, {}, 1.0, 0.785, 0.0, 50));
    goal[0] = std::numeric_limits<double>::quiet_NaN();
    assert(!trajectory.plan(zero, {}, goal, {}, 0.3, 0.785, 1.0, 50));
    auto velocity = VrJointGuard::Joints{};
    velocity[0] = 1.0;
    assert(!trajectory.plan(zero, velocity, zero, {}, 0.3, 0.785, 1.0, 50));
    velocity[0] = 0.1;
    goal = zero;
    goal[0] = 0.001;
    assert(!trajectory.configure(zero, velocity, goal, velocity, 0.6, 0.785, 1.0, 50));
    assert(trajectory.plan(zero, velocity, goal, {}, 0.034, 0.785, 1.0, 50));
}

int main()
{
    test_invalid_and_infeasible();
    test_retargets_and_settle();
    test_streaming_aligned_chunks();
    std::cout << "policy trajectory tests passed\n";
}
