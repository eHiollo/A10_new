#ifndef A10_POLICY_LOG_STATE_HPP_
#define A10_POLICY_LOG_STATE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>

namespace a10_tcp
{
// A single atomic word keeps the session and its flags consistent for TCP readers.
// All RT operations are atomic only; file creation and writes remain in Python.
class PolicyLogState
{
public:
    struct Snapshot
    {
        std::uint64_t session;
        bool running;
        bool enabled;
    };

    std::uint64_t prepare(bool enabled)
    {
        const auto session = next_session_.fetch_add(1, std::memory_order_relaxed);
        return (session << 2) | 2U | (enabled ? 1U : 0U);
    }

    void start(std::uint64_t token) { state_.store(token, std::memory_order_release); }

    void finish(std::uint64_t token)
    {
        // A late collectNrt for an old plan must not stop a newer session.
        state_.compare_exchange_strong(token, token & ~std::uint64_t{3},
                                       std::memory_order_acq_rel);
    }

    void stop() { state_.fetch_and(~std::uint64_t{3}, std::memory_order_acq_rel); }

    Snapshot snapshot() const
    {
        const auto value = state_.load(std::memory_order_acquire);
        return {value >> 2, (value & 2U) != 0, (value & 1U) != 0};
    }

private:
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "Policy log state must be lock-free on the robot");
    std::atomic<std::uint64_t> next_session_{
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count())};
    std::atomic<std::uint64_t> state_{0};
};
}  // namespace a10_tcp

#endif
