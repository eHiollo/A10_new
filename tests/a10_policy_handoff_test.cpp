#include "a10_policy_batch.hpp"

#include <cassert>
#include <iomanip>
#include <iostream>
#include <string>

using Queue = a10_tcp::PolicyBatchQueue;

Queue::Actions trajectory(std::size_t count, double step = 0.001)
{
    Queue::Actions actions;
    for (std::size_t index = 0; index < count; ++index)
        actions.emplace_back(7, index * step);
    return actions;
}

void test_handoff()
{
    Queue queue;
    const auto now = Queue::Clock::now();
    const std::vector<double> command(7, 0.0);
    auto sequence = queue.replace(trajectory(30), 0.3, now, std::nullopt, true);
    assert(queue.align(sequence, 1.0 / 30.0, nullptr, now) == 9);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50));
    auto status = queue.status();
    assert(status.handoff_skip > 0 && status.handoff_skip < 20);
    assert(status.remaining == static_cast<std::size_t>(21 - status.handoff_skip));
    assert(!status.handoff_fallback);
    assert(status.handoff_late_s < 0.0021);
    std::vector<double> goal;
    assert(queue.peek(goal, sequence));
    a10_tcp::PolicyTrajectory planned;
    assert(planned.configure(command, {}, goal, status.handoff_velocity,
        status.handoff_duration_s, 0.785, 1.0, 50));
    assert(queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50));
    assert(queue.status().remaining == status.remaining);
    assert(queue.update_remaining_time(sequence, command, 0.785, 50));
    assert(queue.status().remaining_time_s == -1.0);

    sequence = queue.replace(trajectory(30), 0.31, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50);
    assert(!queue.status().handoff_fallback);
    assert(queue.status().handoff_late_s < 0.0021);

    sequence = queue.replace(trajectory(30), 0.8, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50));
    assert(queue.status().handoff_fallback);
    assert(queue.status().remaining == 6 && queue.status().handoff_skip == 0);

    sequence = queue.replace(trajectory(30, 0.01), 0.3, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    queue.select_handoff(sequence, 1.0 / 30.0, command, 0.075, 50);
    status = queue.status();
    assert(status.remaining == 21 && status.handoff_fallback && status.handoff_skip == 0);
    assert(queue.peek(goal, sequence) && std::abs(goal[0] - 0.09) < 1e-9);
    assert(!queue.select_handoff(sequence - 1, 1.0 / 30.0, command, 0.785, 50));
    assert(!queue.select_handoff(sequence, 0.0, command, 0.785, 50));

    sequence = queue.replace(trajectory(30, 0.02), 0.3, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    queue.select_handoff(sequence, 1.0 / 30.0, command, 0.075, 50, 0.25, 85.0);
    assert(queue.peek(goal, sequence) && goal[0] <= 0.25);
    assert(queue.status().prefix_skip + queue.status().handoff_skip == 9);

    sequence = queue.replace(trajectory(30), 0.3, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50, 0.005, 85.0);
    assert(!queue.status().handoff_selected && queue.status().remaining == 21);

    sequence = queue.replace(trajectory(30), 1.1, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50));
    assert(queue.status().expired && queue.status().remaining == 0);

    sequence = queue.replace(trajectory(30), 0.0, now);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50));
    assert(!queue.status().handoff_selected && queue.status().remaining == 30);
    queue.clear();
    assert(!queue.status().handoff_selected);
}

void test_velocity_and_gripper_feasibility()
{
    Queue queue;
    const auto now = Queue::Clock::now();
    const std::vector<double> command(7, 0.0);
    a10_tcp::PolicyTrajectory::Velocity velocity{};
    velocity[0] = -0.5;
    auto sequence = queue.replace(trajectory(30), 0.3, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, command, 0.785, 50, 0.5, 85, velocity, 1.0));
    assert(queue.status().handoff_fallback && queue.status().handoff_skip == 0);
    std::vector<double> goal;
    assert(queue.peek(goal, sequence));
    a10_tcp::PolicyTrajectory planned;
    assert(planned.configure(command, velocity, goal, {}, queue.status().handoff_duration_s, 0.785, 1.0, 50));

    auto actions = trajectory(30, 0.0);
    for (std::size_t index = 0; index < actions.size(); ++index)
        actions[index][6] = 77.0 - std::min(73.5, index * 3.0);
    auto open_command = command;
    open_command[6] = 77.0;
    sequence = queue.replace(actions, 0.476, now, std::nullopt, true);
    queue.align(sequence, 1.0 / 30.0, nullptr, now);
    assert(queue.select_handoff(sequence, 1.0 / 30.0, open_command, 0.785, 50));
    assert(queue.status().handoff_fallback && queue.status().handoff_skip == 0);
    assert(queue.status().remaining == 16);
    assert(queue.peek(goal, sequence) && goal == actions[14]);
    assert(planned.configure(open_command, {}, goal, {}, queue.status().handoff_duration_s, 0.785, 1.0, 50));
    assert(queue.endpoint_velocity(sequence, 1.0 / 30.0, 0.785, velocity));
    assert(velocity == a10_tcp::PolicyTrajectory::Velocity{});
    assert(!queue.endpoint_velocity(sequence - 1, 1.0 / 30.0, 0.785, velocity));
}

int main(int argc, char** argv)
{
    if (argc == 1)
    {
        test_handoff();
        test_velocity_and_gripper_feasibility();
        std::cout << "policy handoff tests passed\n";
        return 0;
    }
    const bool with_velocity = std::string(argv[1]) == "--replay-velocity";
    if (std::string(argv[1]) != "--replay" && !with_velocity)
        return 1;
    double age, arm_vel, grip_vel;
    std::size_t count;
    std::cout << std::setprecision(17);
    while (std::cin >> age >> arm_vel >> grip_vel >> count)
    {
        std::vector<double> command(7);
        for (auto& value : command)
            std::cin >> value;
        a10_tcp::PolicyTrajectory::Velocity velocity{};
        if (with_velocity)
            for (auto& value : velocity)
                std::cin >> value;
        Queue::Actions actions(count, std::vector<double>(7));
        for (auto& action : actions)
            for (auto& value : action)
                std::cin >> value;
        if (!std::cin)
            return 2;
        Queue queue;
        const auto now = Queue::Clock::now();
        const auto sequence = queue.replace(actions, age, now, std::nullopt, true);
        queue.align(sequence, 1.0 / 30.0, nullptr, now);
        if (!queue.select_handoff(sequence, 1.0 / 30.0, command, arm_vel, grip_vel, 0.5, 85.0, velocity, 1.0))
            return 4;
        const auto status = queue.status();
        std::vector<double> goal;
        if (!queue.peek(goal, sequence))
            return 3;
        std::cout << status.prefix_skip << ' ' << status.handoff_skip << ' '
                  << status.handoff_duration_s << ' ' << status.handoff_late_s;
        for (double value : goal)
            std::cout << ' ' << value;
        if (with_velocity)
            std::cout << ' ' << status.handoff_fallback;
        std::cout << '\n';
    }
}
