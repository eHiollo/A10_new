#ifndef A10_POLICY_JERK_TRAJECTORY_HPP_
#define A10_POLICY_JERK_TRAJECTORY_HPP_

#include "a10_policy_trajectory.hpp"

namespace a10_tcp
{
class PolicyJerkTrajectory
{
public:
    using Joints = VrJointGuard::Joints;
    static constexpr double dt = VrJointGuard::dt;
    static constexpr double max_duration = 10.0;

    bool configure(const std::vector<double>& start, const Joints& velocity, const Joints& acceleration,
                   const std::vector<double>& goal, const Joints& end_velocity, double duration,
                   double arm_vel, double arm_acc, double arm_jerk, double grip_vel)
    {
        if (start.size() != 7 || goal.size() != 7 || !std::isfinite(duration) || duration < dt ||
            duration > max_duration || !positive(arm_vel) || !positive(arm_acc) ||
            !positive(arm_jerk) || !positive(grip_vel))
            return false;
        const int ticks = static_cast<int>(std::ceil(duration / dt - 1e-9));
        const double span = ticks * dt;
        const double ratio = dt / span;
        const double history_span = span + 2.0 * dt;
        std::array<Joints, 6> coefficients{};
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
            if (!std::isfinite(velocity[axis]) || !std::isfinite(acceleration[axis]) ||
                !std::isfinite(end_velocity[axis]))
                return false;
            const double base_velocity = (velocity[axis] + 0.5 * acceleration[axis] * dt) * span;
            const double base_acceleration = acceleration[axis] * span * span;
            const double factor = 1.0 + 3.0 * ratio + 2.0 * ratio * ratio;
            const double factor_first = 3.0 + 6.0 * ratio + 2.0 * ratio * ratio;
            const double factor_second = 6.0 + 6.0 * ratio;
            const double residual = (distance - base_velocity - 0.5 * base_acceleration) / factor;
            const double residual_first = (end_velocity[axis] * span - base_velocity - base_acceleration
                - factor_first * residual) / factor;
            const double residual_second = (-base_acceleration - factor_second * residual
                - 2.0 * factor_first * residual_first) / factor;
            const double tail_quadratic = 0.5 * residual_second;
            const double tail_linear = residual_first - 2.0 * tail_quadratic;
            const double tail_constant = residual - tail_linear - tail_quadratic;
            auto& power = coefficients[axis];
            power = {0.0, base_velocity + 2.0 * ratio * ratio * tail_constant,
                0.5 * base_acceleration + 3.0 * ratio * tail_constant + 2.0 * ratio * ratio * tail_linear,
                tail_constant + 3.0 * ratio * tail_linear + 2.0 * ratio * ratio * tail_quadratic,
                tail_linear + 3.0 * ratio * tail_quadratic, tail_quadratic};
            if (!std::all_of(power.begin(), power.end(), [](double value) { return std::isfinite(value); }))
                return false;
            const auto controls = bernstein(power, -2.0 * ratio, 1.0 + 2.0 * ratio);
            const auto speeds = derivative(controls, history_span);
            const auto accelerations = derivative(speeds, history_span);
            const auto jerks = derivative(accelerations, history_span);
            const double speed_limit = std::min(arm_vel, VrJointGuard::speed_limit(axis));
            const double acceleration_limit = std::min(arm_acc, VrJointGuard::acceleration);
            if (!bounded(speeds, -speed_limit, speed_limit) ||
                !bounded(accelerations, -acceleration_limit, acceleration_limit) ||
                !bounded(jerks, -arm_jerk, arm_jerk))
                return false;
            const auto forward_speeds = derivative(bernstein(power, 0.0, 1.0), span);
            int previous_sign = 0;
            int turns = 0;
            for (double speed : forward_speeds)
            {
                const int sign = speed > 1e-10 ? 1 : (speed < -1e-10 ? -1 : 0);
                if (sign != 0)
                {
                    turns += previous_sign != 0 && sign != previous_sign;
                    previous_sign = sign;
                }
            }
            if (turns > 1 && !bounded(forward_speeds, 0.0, speed_limit) &&
                !bounded(forward_speeds, -speed_limit, 0.0))
                return false;
        }
        start_ = start;
        goal_ = goal;
        coefficients_ = coefficients;
        end_velocity_ = end_velocity;
        ticks_ = ticks;
        return true;
    }

    bool plan(const std::vector<double>& start, const Joints& velocity, const Joints& acceleration,
              const std::vector<double>& goal, const Joints& end_velocity, double minimum_duration,
              double arm_vel, double arm_acc, double arm_jerk, double grip_vel)
    {
        if (!std::isfinite(minimum_duration) || minimum_duration < 0.0)
            return false;
        double span = std::max(dt, minimum_duration);
        for (int attempt = 0; attempt < 64 && span <= max_duration; ++attempt)
        {
            if (configure(start, velocity, acceleration, goal, end_velocity, span,
                arm_vel, arm_acc, arm_jerk, grip_vel))
                return true;
            span = std::ceil(std::max(span + dt, span * 1.15) / dt) * dt;
        }
        return false;
    }

    bool brake(const std::vector<double>& start, const Joints& velocity, const Joints& acceleration,
               double arm_vel, double arm_acc, double arm_jerk, double grip_vel)
    {
        if (start.size() != 7)
            return false;
        double span = 0.034;
        for (int attempt = 0; attempt < 64 && span <= max_duration; ++attempt)
        {
            auto goal = start;
            for (std::size_t axis = 0; axis < 6; ++axis)
                goal[axis] += 0.5 * velocity[axis] * span + acceleration[axis] * span * span / 12.0;
            if (configure(start, velocity, acceleration, goal, {}, span, arm_vel, arm_acc, arm_jerk, grip_vel))
                return true;
            span = std::ceil(span * 1.15 / dt) * dt;
        }
        return false;
    }

    std::vector<double> sample(int tick) const
    {
        if (tick >= ticks_)
            return goal_;
        const double phase = std::clamp(tick, -2, ticks_) * dt / duration();
        auto command = start_;
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            double offset = coefficients_[axis][5];
            for (int degree = 4; degree >= 0; --degree)
                offset = offset * phase + coefficients_[axis][degree];
            command[axis] += offset;
        }
        command[6] += (goal_[6] - start_[6]) * std::max(0.0, phase);
        return command;
    }

    int ticks() const { return ticks_; }
    double duration() const { return ticks_ * dt; }
    const Joints& end_velocity() const { return end_velocity_; }

private:
    static bool positive(double value) { return std::isfinite(value) && value > 0.0; }

    static double binomial(int degree, int index)
    {
        double value = 1.0;
        for (int factor = 1; factor <= index; ++factor)
            value *= static_cast<double>(degree - factor + 1) / factor;
        return value;
    }

    static Joints bernstein(const Joints& power, double origin, double scale)
    {
        Joints shifted{}, controls{};
        Joints origin_powers{}, scale_powers{};
        origin_powers[0] = scale_powers[0] = 1.0;
        for (int degree = 1; degree <= 5; ++degree)
        {
            origin_powers[degree] = origin_powers[degree - 1] * origin;
            scale_powers[degree] = scale_powers[degree - 1] * scale;
        }
        for (int degree = 0; degree <= 5; ++degree)
            for (int index = 0; index <= degree; ++index)
                shifted[index] += power[degree] * binomial(degree, index)
                    * origin_powers[degree - index] * scale_powers[index];
        for (int degree = 0; degree <= 5; ++degree)
            for (int index = 0; index <= degree; ++index)
                controls[degree] += shifted[index] * binomial(degree, index) / binomial(5, index);
        return controls;
    }

    template<std::size_t Size>
    static std::array<double, Size - 1> derivative(const std::array<double, Size>& controls, double span)
    {
        std::array<double, Size - 1> result{};
        for (std::size_t index = 0; index + 1 < Size; ++index)
            result[index] = (Size - 1) * (controls[index + 1] - controls[index]) / span;
        return result;
    }

    template<std::size_t Size>
    static bool bounded(const std::array<double, Size>& controls, double lower, double upper, int depth = 0)
    {
        bool inside = true;
        for (double value : controls)
        {
            if (!std::isfinite(value))
                return false;
            inside = inside && value >= lower - 1e-10 && value <= upper + 1e-10;
        }
        if (inside)
            return true;
        if (controls.front() < lower - 1e-10 || controls.front() > upper + 1e-10 ||
            controls.back() < lower - 1e-10 || controls.back() > upper + 1e-10 || depth >= 6)
            return false;
        auto work = controls;
        std::array<double, Size> left{}, right{};
        left.front() = work.front();
        right.back() = work.back();
        for (std::size_t level = 1; level < Size; ++level)
        {
            for (std::size_t index = 0; index < Size - level; ++index)
                work[index] = 0.5 * (work[index] + work[index + 1]);
            left[level] = work.front();
            right[Size - 1 - level] = work[Size - 1 - level];
        }
        return bounded(left, lower, upper, depth + 1) && bounded(right, lower, upper, depth + 1);
    }

    std::vector<double> start_ = std::vector<double>(7, 0.0);
    std::vector<double> goal_ = std::vector<double>(7, 0.0);
    std::array<Joints, 6> coefficients_{};
    Joints end_velocity_{};
    int ticks_{1};
};
}

#endif
