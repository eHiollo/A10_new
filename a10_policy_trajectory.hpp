#ifndef A10_POLICY_TRAJECTORY_HPP_
#define A10_POLICY_TRAJECTORY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include "a10_vr_joint_guard.hpp"

namespace a10_tcp
{
class PolicyTrajectory
{
public:
    using Velocity = VrJointGuard::Joints;
    static constexpr double dt = VrJointGuard::dt;

    bool configure(const std::vector<double>& start, const Velocity& velocity,
                   const std::vector<double>& goal, const Velocity& end_velocity,
                   double duration, double arm_vel, double arm_acc, double grip_vel)
    {
        if (start.size() != 7 || goal.size() != 7 || !std::isfinite(duration) || duration < dt ||
            duration > 1e6 || !std::isfinite(arm_vel) || arm_vel <= 0.0 ||
            !std::isfinite(arm_acc) || arm_acc <= 0.0 || !std::isfinite(grip_vel) || grip_vel <= 0.0)
            return false;
        const int ticks = static_cast<int>(std::ceil(duration / dt - 1e-9));
        const double span = ticks * dt;
        const double acceleration_limit = std::min(arm_acc, VrJointGuard::acceleration);
        std::array<double, 6> quadratic{}, cubic{};
        for (std::size_t axis = 0; axis < 7; ++axis)
        {
            if (!std::isfinite(start[axis]) || !std::isfinite(goal[axis]))
                return false;
            const double distance = goal[axis] - start[axis];
            if (axis == 6)
            {
                if (std::abs(distance) / span > grip_vel + 1e-9)
                    return false;
                continue;
            }
            if (!std::isfinite(velocity[axis]) || !std::isfinite(end_velocity[axis]))
                return false;
            quadratic[axis] = (3.0 * distance / span - 2.0 * velocity[axis] - end_velocity[axis]) / span;
            cubic[axis] = (-2.0 * distance / span + velocity[axis] + end_velocity[axis]) / (span * span);
            const double initial_acceleration = 2.0 * quadratic[axis];
            const double final_acceleration = initial_acceleration + 6.0 * cubic[axis] * span;
            const double speed_limit = std::min(arm_vel, VrJointGuard::speed_limit(axis));
            double peak_speed = std::max(std::abs(velocity[axis]), std::abs(end_velocity[axis]));
            if (std::abs(cubic[axis]) > 1e-15)
            {
                const double extremum = -quadratic[axis] / (3.0 * cubic[axis]);
                if (extremum > 0.0 && extremum < span)
                {
                    const double interior_velocity = velocity[axis]
                        + extremum * (2.0 * quadratic[axis] + 3.0 * cubic[axis] * extremum);
                    if (velocity[axis] * end_velocity[axis] >= 0.0 &&
                        interior_velocity * end_velocity[axis] < -1e-12)
                        return false;
                    peak_speed = std::max(peak_speed, std::abs(interior_velocity));
                }
            }
            if (!std::isfinite(peak_speed) || !std::isfinite(initial_acceleration) || !std::isfinite(final_acceleration) ||
                peak_speed > speed_limit + 1e-9 ||
                std::max(std::abs(initial_acceleration), std::abs(final_acceleration)) > acceleration_limit + 1e-9)
                return false;
        }
        start_ = start;
        goal_ = goal;
        velocity_ = velocity;
        end_velocity_ = end_velocity;
        quadratic_ = quadratic;
        cubic_ = cubic;
        ticks_ = ticks;
        return true;
    }

    bool plan(const std::vector<double>& start, const Velocity& velocity,
              const std::vector<double>& goal, const Velocity& end_velocity,
              double minimum_duration, double arm_vel, double arm_acc, double grip_vel)
    {
        if (!std::isfinite(minimum_duration) || minimum_duration < 0.0)
            return false;
        double span = std::max(dt, minimum_duration);
        for (int attempt = 0; attempt < 80 && span <= 1e6; ++attempt)
        {
            if (configure(start, velocity, goal, end_velocity, span, arm_vel, arm_acc, grip_vel))
                return true;
            span = std::ceil(std::max(span + dt, span * 1.15) / dt) * dt;
        }
        return false;
    }

    std::vector<double> sample(int tick) const
    {
        const double elapsed = std::clamp(tick, 0, ticks_) * dt;
        if (tick >= ticks_)
            return goal_;
        auto command = start_;
        for (std::size_t axis = 0; axis < 6; ++axis)
            command[axis] += elapsed * (velocity_[axis] + elapsed * (quadratic_[axis] + elapsed * cubic_[axis]));
        command[6] += (goal_[6] - start_[6]) * elapsed / duration();
        return command;
    }

    int ticks() const { return ticks_; }
    double duration() const { return ticks_ * dt; }
    const Velocity& end_velocity() const { return end_velocity_; }

private:
    std::vector<double> start_ = std::vector<double>(7, 0.0);
    std::vector<double> goal_ = std::vector<double>(7, 0.0);
    Velocity velocity_{}, end_velocity_{}, quadratic_{}, cubic_{};
    int ticks_{1};
};

inline PolicyTrajectory::Velocity policy_endpoint_velocity(const std::vector<double>& previous,
    const std::vector<double>& goal, const std::vector<double>* next, double step_s, double arm_vel)
{
    PolicyTrajectory::Velocity velocity{};
    if (next == nullptr)
        return velocity;
    for (std::size_t axis = 0; axis < velocity.size(); ++axis)
    {
        const double incoming = (goal[axis] - previous[axis]) / step_s;
        const double outgoing = ((*next)[axis] - goal[axis]) / step_s;
        if ((incoming > 0.0 && outgoing > 0.0) || (incoming < 0.0 && outgoing < 0.0))
        {
            const double slope = 2.0 * std::min(std::abs(incoming), std::abs(outgoing)) /
                (1.0 + std::min(std::abs(incoming), std::abs(outgoing)) / std::max(std::abs(incoming), std::abs(outgoing)));
            velocity[axis] = std::copysign(std::min({slope, arm_vel, VrJointGuard::speed_limit(axis)}), outgoing);
        }
    }
    return velocity;
}
}

#endif
