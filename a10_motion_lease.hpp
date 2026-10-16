#pragma once
#include <atomic>
#include <memory>
#include <stdexcept>

namespace a10_follower {
enum class MotionOwner { None, Follower, Policy, VrVel, Vr };
inline std::atomic<MotionOwner> motion_owner{MotionOwner::None};
class MotionLease {
public:
    static std::shared_ptr<MotionLease> acquire(MotionOwner requested) {
        auto lease = std::shared_ptr<MotionLease>(new MotionLease(requested));
        auto expected = MotionOwner::None;
        if (!motion_owner.compare_exchange_strong(expected, requested))
            throw std::runtime_error("another follower/policy/VR motion owns the controller; stop it first");
        lease->owned_ = true;
        return lease;
    }
    ~MotionLease() {
        if (owned_) { auto expected = owner_; motion_owner.compare_exchange_strong(expected, MotionOwner::None); }
    }
    MotionLease(const MotionLease&) = delete;
    MotionLease& operator=(const MotionLease&) = delete;
private:
    explicit MotionLease(MotionOwner value) : owner_(value) {}
    MotionOwner owner_; bool owned_{false};
};
} // namespace a10_follower
