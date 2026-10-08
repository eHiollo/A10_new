#ifndef A10_POLICY_SMOOTHER_HPP_
#define A10_POLICY_SMOOTHER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace a10_tcp
{
class PolicyCommandSmoother
{
public:
    void reset(const std::vector<double>& command)
    {
        position_ = command;
        velocity_.fill(0.0);
    }

    const std::vector<double>& step(const std::vector<double>& reference, double max_velocity, double max_acceleration)
    {
        constexpr double dt = 0.002;
        const double velocity_change = max_acceleration * dt;
        for (std::size_t axis = 0; axis < velocity_.size(); ++axis)
        {
            const double error = reference[axis] - position_[axis];
            const double braking_velocity = std::sqrt(velocity_change * velocity_change
                + 2.0 * max_acceleration * std::abs(error)) - velocity_change;
            const double desired_velocity = std::copysign(std::min(max_velocity, braking_velocity), error);
            velocity_[axis] += std::clamp(desired_velocity - velocity_[axis], -velocity_change, velocity_change);
            position_[axis] += velocity_[axis] * dt;
        }
        position_[6] = reference[6];
        return position_;
    }

    const std::vector<double>& position() const { return position_; }

    double lag(const std::vector<double>& reference) const
    {
        double result = 0.0;
        for (std::size_t axis = 0; axis < velocity_.size(); ++axis)
            result = std::max(result, std::abs(reference[axis] - position_[axis]));
        return result;
    }

    bool settled(const std::vector<double>& reference) const
    {
        return lag(reference) < 1e-6 && std::all_of(velocity_.begin(), velocity_.end(),
            [](double velocity) { return std::abs(velocity) < 1e-5; });
    }

private:
    std::vector<double> position_ = std::vector<double>(7, 0.0);
    std::array<double, 6> velocity_{};
};
}

#endif
