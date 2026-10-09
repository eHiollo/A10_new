#ifndef A10_POLICY_BATCH_HPP_
#define A10_POLICY_BATCH_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include "a10_vr_joint_guard.hpp"
#include "a10_policy_trajectory.hpp"
#include "a10_policy_jerk_trajectory.hpp"

namespace a10_tcp
{
inline int policy_blend_steps(const std::vector<double>& start, const std::vector<double>& goal,
                              double arm_vel, double grip_vel)
{
    int steps = 1;
    for (std::size_t axis = 0; axis < 7; ++axis)
    {
        const double velocity = axis < 6 ? std::min(arm_vel, VrJointGuard::speed_limit(axis)) : grip_vel;
        const double required = std::ceil(std::abs(goal[axis] - start[axis]) / (velocity * 0.002));
        if (!std::isfinite(required) || required >= std::numeric_limits<int>::max() - 1)
            return std::numeric_limits<int>::max();
        steps = std::max(steps, static_cast<int>(required) + 1);
    }
    return steps;
}

inline int policy_min_segment_steps(bool timed, double handoff_s = 0.0)
{
    if (!timed)
        return 2;
    const double duration = std::max(1.0 / 30.0, std::isfinite(handoff_s) ? std::clamp(handoff_s, 0.0, 1.0) : 0.0);
    return static_cast<int>(std::ceil(duration / 0.002)) + 1;
}

struct PolicyBatchStatus
{
    std::uint64_t sequence{0};
    std::size_t remaining{0};
    std::size_t received{0};
    int prefix_skip{0};
    double alignment_age_s{0.0};
    bool aligned{false};
    bool expired{false};
    bool smooth{false};
    bool handoff_selected{false};
    bool handoff_fallback{false};
    bool planned_trajectory{false};
    bool jerk_limited{false};
    bool plan_rejected{false};
    int handoff_skip{0};
    double handoff_duration_s{0.0};
    double handoff_target_time_s{0.0};
    double handoff_late_s{0.0};
    PolicyTrajectory::Velocity handoff_velocity{};
    double remaining_time_s{-1.0};
};

class PolicyBatchQueue
{
public:
    using Clock = std::chrono::steady_clock;
    using Actions = std::deque<std::vector<double>>;

    std::uint64_t replace(Actions actions, double age_s, Clock::time_point now = Clock::now(),
                          std::optional<std::uint64_t> expected_sequence = std::nullopt, bool smooth = false)
    {
        if (!std::isfinite(age_s) || age_s < 0.0 || actions.empty())
            throw std::invalid_argument("empty batch or invalid observation age");
        for (const auto& row : actions)
        {
            if (row.size() != 7 || !std::all_of(row.begin(), row.end(), [](double value) { return std::isfinite(value); }))
                throw std::invalid_argument("actions must contain exactly seven finite numbers per row");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (expected_sequence && (status_.sequence != *expected_sequence || !actions_.empty()))
            throw std::invalid_argument("batch changed or queue still executing");
        const auto sequence = status_.sequence + 1;
        actions_ = std::move(actions);
        previous_action_.clear();
        handoff_pending_ = false;
        status_ = PolicyBatchStatus{};
        status_.sequence = sequence;
        status_.received = actions_.size();
        status_.smooth = smooth;
        received_at_ = now;
        observation_age_s_ = age_s;
        return sequence;
    }

    PolicyBatchStatus status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = status_;
        result.remaining = actions_.size();
        if (actions_.empty())
            result.remaining_time_s = 0.0;
        else if (status_.remaining_time_s >= 0.0)
            result.remaining_time_s = std::max(0.0, status_.remaining_time_s - std::chrono::duration<double>(Clock::now() - estimated_at_).count());
        return result;
    }

    bool update_remaining_time(std::uint64_t expected_sequence, const std::vector<double>& command,
                               double arm_vel, double grip_vel, Clock::time_point now = Clock::now())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_sequence || !status_.aligned || command.size() != 7 ||
            !std::all_of(command.begin(), command.end(), [](double value) { return std::isfinite(value); }) ||
            !std::isfinite(arm_vel) || !std::isfinite(grip_vel) || arm_vel <= 0.0 || grip_vel <= 0.0)
            return false;
        if (status_.planned_trajectory)
        {
            status_.remaining_time_s = -1.0;
            return true;
        }
        double remaining = 0.0;
        const auto* previous = &command;
        for (const auto& goal : actions_)
        {
            const double handoff_s = previous == &command && actions_.size() == status_.received - status_.prefix_skip - status_.handoff_skip
                ? (status_.handoff_selected ? status_.handoff_duration_s : status_.alignment_age_s) : 0.0;
            remaining += std::max(policy_min_segment_steps(status_.smooth, handoff_s),
                policy_blend_steps(*previous, goal, arm_vel, grip_vel)) * 0.002;
            previous = &goal;
        }
        status_.remaining_time_s = remaining;
        estimated_at_ = now;
        return true;
    }

    void mark_planned(std::uint64_t expected_seq, bool jerk_limited = false)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence == expected_seq)
        {
            status_.planned_trajectory = true;
            status_.jerk_limited = jerk_limited;
            status_.remaining_time_s = -1.0;
        }
    }

    bool reject(std::uint64_t expected_seq)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_seq)
            return false;
        actions_.clear();
        previous_action_.clear();
        handoff_pending_ = false;
        status_.plan_rejected = true;
        status_.remaining_time_s = 0.0;
        return true;
    }

    bool peek(std::vector<double>& out, std::uint64_t expected_seq) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_seq || actions_.empty())
            return false;
        out = actions_.front();
        return true;
    }

    bool pop(std::uint64_t expected_seq)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_seq || actions_.empty())
            return false;
        previous_action_ = actions_.front();
        actions_.pop_front();
        handoff_pending_ = false;
        return true;
    }

    bool endpoint_velocity(std::uint64_t expected_seq, double step_s, double arm_vel,
                           PolicyTrajectory::Velocity& velocity) const
    {
        if (!std::isfinite(step_s) || step_s <= 0.0 || !std::isfinite(arm_vel) || arm_vel <= 0.0)
            return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_seq || actions_.empty())
            return false;
        velocity = handoff_pending_ ? status_.handoff_velocity : policy_endpoint_velocity(
            previous_action_.empty() ? actions_.front() : previous_action_, actions_.front(),
            actions_.size() > 1 ? &actions_[1] : nullptr, step_s, arm_vel);
        return true;
    }

    int align(std::uint64_t expected_seq, double step_s, double* age_out, Clock::time_point now = Clock::now())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_seq || !std::isfinite(step_s) || step_s <= 0.0)
            return -1;
        if (!status_.aligned)
        {
            const double elapsed = std::max(0.0, std::chrono::duration<double>(now - received_at_).count());
            const double age = observation_age_s_ + elapsed;
            const double skipped = std::floor((age + 1e-9) / step_s);
            const auto skip = static_cast<std::size_t>(std::min(skipped, static_cast<double>(actions_.size())));
            status_.alignment_age_s = age;
            status_.prefix_skip = static_cast<int>(skip);
            status_.aligned = true;
            status_.expired = status_.received > 0 && skip == actions_.size();
            for (std::size_t index = 0; index < skip; ++index)
            {
                previous_action_ = actions_.front();
                actions_.pop_front();
            }
        }
        if (age_out != nullptr)
            *age_out = status_.alignment_age_s;
        return status_.prefix_skip;
    }

    bool select_handoff(std::uint64_t expected_seq, double step_s, const std::vector<double>& command,
                        double arm_vel, double grip_vel, double max_axis_delta = 0.5, double max_grip_delta = 85.0,
                        const PolicyTrajectory::Velocity& velocity = {}, double arm_acc = 1.0,
                        const PolicyTrajectory::Velocity& acceleration = {}, double arm_jerk = 0.0)
    {
        if (!std::isfinite(step_s) || step_s <= 0.0 || command.size() != 7 ||
            !std::all_of(command.begin(), command.end(), [](double value) { return std::isfinite(value); }) ||
            !std::isfinite(arm_vel) || arm_vel <= 0.0 || !std::isfinite(grip_vel) || grip_vel <= 0.0 ||
            !std::isfinite(max_axis_delta) || max_axis_delta <= 0.0 || !std::isfinite(max_grip_delta) || max_grip_delta <= 0.0)
            return false;
        if (!std::isfinite(arm_acc) || arm_acc <= 0.0 || !std::isfinite(arm_jerk) || arm_jerk < 0.0 ||
            !std::all_of(acceleration.begin(), acceleration.end(), [](double value) { return std::isfinite(value); }) ||
            !std::all_of(velocity.begin(), velocity.end(), [](double value) { return std::isfinite(value); }))
            return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.sequence != expected_seq || !status_.aligned)
            return false;
        if (status_.handoff_selected || !status_.smooth || actions_.empty())
            return true;
        PolicyTrajectory trajectory;
        PolicyJerkTrajectory jerk_trajectory;
        const bool jerk_limited = arm_jerk > 0.0;
        std::size_t selected = 0;
        bool reachable = false;
        for (std::size_t index = 0; index < actions_.size(); ++index)
        {
            bool exceeds = false;
            for (std::size_t axis = 0; axis < 7; ++axis)
                exceeds = exceeds || std::abs(actions_[index][axis] - command[axis]) > (axis < 6 ? max_axis_delta : max_grip_delta);
            if (exceeds)
            {
                if (index == 0)
                    return true;
                break;
            }
            const double available = (status_.prefix_skip + index) * step_s - status_.alignment_age_s;
            if (available + 1e-9 < step_s)
                continue;
            const auto end_velocity = policy_endpoint_velocity(index > 0 ? actions_[index - 1] :
                (previous_action_.empty() ? actions_[index] : previous_action_), actions_[index],
                index + 1 < actions_.size() ? &actions_[index + 1] : nullptr, step_s, arm_vel);
            const bool feasible = jerk_limited
                ? jerk_trajectory.configure(command, velocity, acceleration, actions_[index], end_velocity,
                    available, arm_vel, arm_acc, arm_jerk, grip_vel)
                : trajectory.configure(command, velocity, actions_[index], end_velocity,
                    available, arm_vel, arm_acc, grip_vel);
            if (feasible)
            {
                selected = index;
                reachable = true;
                break;
            }
        }
        if (!reachable)
        {
            const bool feasible = jerk_limited
                ? jerk_trajectory.plan(command, velocity, acceleration, actions_.front(), {}, step_s,
                    arm_vel, arm_acc, arm_jerk, grip_vel)
                : trajectory.plan(command, velocity, actions_.front(), {}, step_s, arm_vel, arm_acc, grip_vel);
            if (!feasible)
                return false;
        }
        status_.handoff_selected = true;
        status_.planned_trajectory = true;
        status_.jerk_limited = jerk_limited;
        status_.handoff_fallback = !reachable;
        status_.handoff_skip = static_cast<int>(selected);
        status_.handoff_target_time_s = (status_.prefix_skip + selected) * step_s;
        status_.handoff_duration_s = jerk_limited ? jerk_trajectory.duration() : trajectory.duration();
        status_.handoff_velocity = jerk_limited ? jerk_trajectory.end_velocity() : trajectory.end_velocity();
        status_.handoff_late_s = std::max(0.0, status_.alignment_age_s + status_.handoff_duration_s - status_.handoff_target_time_s);
        handoff_pending_ = true;
        for (std::size_t index = 0; index < selected; ++index)
        {
            previous_action_ = actions_.front();
            actions_.pop_front();
        }
        status_.remaining_time_s = -1.0;
        return true;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto sequence = status_.sequence + 1;
        actions_.clear();
        previous_action_.clear();
        handoff_pending_ = false;
        status_ = PolicyBatchStatus{};
        status_.sequence = sequence;
        observation_age_s_ = 0.0;
        received_at_ = Clock::now();
    }

private:
    mutable std::mutex mutex_;
    Actions actions_;
    std::vector<double> previous_action_;
    bool handoff_pending_{false};
    PolicyBatchStatus status_;
    Clock::time_point received_at_{Clock::now()};
    Clock::time_point estimated_at_{};
    double observation_age_s_{0.0};
};
}

#endif
