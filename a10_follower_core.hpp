#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include "a10_vr_joint_guard.hpp"

namespace a10_follower {
using Joints = a10_tcp::VrJointGuard::Joints;
using Guard = a10_tcp::VrJointGuard;
struct Limits {
    double max_vel{0.3}, max_acc{1.5}, max_dec{1.5};
    void validate() const {
        if (!std::isfinite(max_vel) || max_vel <= 0 || max_vel > Guard::speed_limit(5)
            || !std::isfinite(max_acc) || max_acc <= 0 || max_acc > Guard::acceleration
            || !std::isfinite(max_dec) || max_dec <= 0 || max_dec > Guard::acceleration)
            throw std::invalid_argument("follower limits must be finite and positive; max_vel <= 0.785398 rad/s, max_acc/max_dec <= 3.490659 rad/s^2");
    }
    std::array<double, 3> values() const { return {max_vel, max_acc, max_dec}; }
};

// Single writer (serialized in NRT), bounded single-attempt reader in RT.
// Every field is atomic: unlike a seqlock over ordinary doubles, this has no
// C++ data race even if publication overlaps a read. SC preserves the snapshot.
template<std::size_t N> class Mailbox {
public:
    struct Snapshot { std::array<double, N> values{}; std::uint64_t token{}, stamp{}, version{}; };
    void publish(const std::array<double, N>& values, std::uint64_t token = 0, std::uint64_t stamp = 0) noexcept {
        sequence_.fetch_add(1);
        for (std::size_t i = 0; i < N; ++i) values_[i].store(values[i]);
        token_.store(token); stamp_.store(stamp);
        sequence_.fetch_add(1);
    }
    bool read(Snapshot& result) const noexcept {
        Snapshot value;
        const auto before = sequence_.load();
        if (before == 0 || (before & 1U)) return false;
        for (std::size_t i = 0; i < N; ++i) value.values[i] = values_[i].load();
        value.token = token_.load(); value.stamp = stamp_.load();
        if (before != sequence_.load()) return false;
        value.version = before; result = value; return true;
    }
private:
    static_assert(std::atomic<double>::is_always_lock_free && std::atomic<std::uint64_t>::is_always_lock_free,
                  "Follower snapshots must be lock-free");
    std::array<std::atomic<double>, N> values_{};
    std::atomic<std::uint64_t> sequence_{0}, token_{0}, stamp_{0};
};

class Tracker {
public:
    bool reset(const Joints& command, const Joints& actual, const Joints& minimum, const Joints& maximum) noexcept {
        guard_.reset();
        if (!guard_.set_position_limits(minimum, maximum)) return false;
        for (std::size_t i = 0; i < 6; ++i)
            if (!std::isfinite(command[i]) || !std::isfinite(actual[i])
                || command[i] <= minimum[i] + 0.002 || command[i] >= maximum[i] - 0.002
                || actual[i] < minimum[i] || actual[i] > maximum[i]
                || std::abs(command[i] - actual[i]) > 0.01) return false;
        position_ = target_ = command; velocity_.fill(0); return true;
    }
    bool target(const Joints& value) noexcept {
        for (std::size_t i = 0; i < 6; ++i)
            if (!std::isfinite(value[i]) || value[i] <= guard_.minimum(i) + 0.002
                || value[i] >= guard_.maximum(i) - 0.002) return false;
        target_ = value; return true;
    }
    void fault(const char* reason) noexcept { guard_.latch(reason); }
    void step(const Joints& actual, const Limits& limits, bool stopping = false) noexcept {
        Joints requested{};
        for (std::size_t i = 0; i < 6; ++i) {
            const double error = target_[i] - position_[i];
            const double dec_step = limits.max_dec * Guard::dt;
            const double braking = std::sqrt(dec_step * dec_step + 2 * limits.max_dec * std::abs(error)) - dec_step;
            double desired = stopping ? 0.0 : std::copysign(std::min({limits.max_vel, Guard::speed_limit(i), braking}), error);
            const bool reversing = desired * velocity_[i] < 0;
            if (reversing) desired = 0;
            const double rate = (reversing || std::abs(desired) < std::abs(velocity_[i])) ? limits.max_dec : limits.max_acc;
            const double next_velocity = velocity_[i] + std::clamp(desired - velocity_[i], -rate * Guard::dt, rate * Guard::dt);
            requested[i] = position_[i] + next_velocity * Guard::dt;
        }
        // Ordinary stop follows max_dec; latched faults use the independent
        // guard's braking priority. Its accepted output seeds the next tick.
        const auto result = guard_.step(position_, velocity_, requested, actual);
        position_ = result.position; velocity_ = result.velocity;
    }
    const Joints& position() const noexcept { return position_; }
    const Joints& velocity() const noexcept { return velocity_; }
    const Joints& target() const noexcept { return target_; }
    bool stopped() const noexcept {
        for (double v : velocity_) if (std::abs(v) > 1e-8) return false;
        return true;
    }
    bool settled() const noexcept {
        if (!stopped()) return false;
        for (std::size_t i = 0; i < 6; ++i) if (std::abs(position_[i] - target_[i]) > 1e-6) return false;
        return true;
    }
    bool faulted() const noexcept { return guard_.faulted(); }
    const char* fault() const noexcept { return guard_.fault(); }
private:
    Guard guard_;
    Joints position_{}, velocity_{}, target_{};
};
} // namespace a10_follower
