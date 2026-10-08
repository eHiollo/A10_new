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

namespace a10_tcp
{
inline int policy_blend_steps(const std::vector<double>& start, const std::vector<double>& goal,
                              double arm_vel, double grip_vel)
{
    int steps = 1;
    for (std::size_t axis = 0; axis < 7; ++axis)
    {
        const double velocity = axis < 6 ? arm_vel : grip_vel;
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
        double remaining = 0.0;
        const auto* previous = &command;
        for (const auto& goal : actions_)
        {
            const double handoff_s = previous == &command && actions_.size() == status_.received - status_.prefix_skip
                ? status_.alignment_age_s : 0.0;
            remaining += std::max(policy_min_segment_steps(status_.smooth, handoff_s),
                policy_blend_steps(*previous, goal, arm_vel, grip_vel)) * 0.002;
            previous = &goal;
        }
        status_.remaining_time_s = remaining;
        estimated_at_ = now;
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
        actions_.pop_front();
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
                actions_.pop_front();
        }
        if (age_out != nullptr)
            *age_out = status_.alignment_age_s;
        return status_.prefix_skip;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto sequence = status_.sequence + 1;
        actions_.clear();
        status_ = PolicyBatchStatus{};
        status_.sequence = sequence;
        observation_age_s_ = 0.0;
        received_at_ = Clock::now();
    }

private:
    mutable std::mutex mutex_;
    Actions actions_;
    PolicyBatchStatus status_;
    Clock::time_point received_at_{Clock::now()};
    Clock::time_point estimated_at_{};
    double observation_age_s_{0.0};
};
}

#endif
