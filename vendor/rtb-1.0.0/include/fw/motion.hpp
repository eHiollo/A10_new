#ifndef RTB_FRAMEWORK_MOTION_HPP
#define RTB_FRAMEWORK_MOTION_HPP

#include "fw/lockable.hpp"
#include "fw/object.hpp"
#include <atomic>

namespace rtb::fw {

/**
 * @brief 运动学模型基类，负责关节读写、正逆解
 * 多任务抢占核心在电机控制权，故锁挂于 Motion
 */
class MotionBase : public ObjectRoot, public LockableResource {
public:
    using ObjectRoot::ObjectRoot;

    ~MotionBase() override = default;

    virtual auto jointNum() const -> int { return 6; }

    virtual auto getJoints(double* q) const -> bool {
        (void)q;
        return false;
    }

    virtual auto setJoints(const double* q) -> bool {
        (void)q;
        return false;
    }

    virtual auto forwardKinematics(const double* q, double* pose) -> bool {
        (void)q;
        (void)pose;
        return false;
    }

    virtual auto inverseKinematics(const double* pose, double* q) -> bool {
        (void)q;
        (void)pose;
        return false;
    }

    // -------------------------------------------------------
    // 资源锁（从 RobotBase 下沉，多任务并发安全）
    // -------------------------------------------------------
    auto try_lock() -> bool override {
        bool expected = false;
        return is_busy_.compare_exchange_strong(expected, true);
    }

    auto unlock() -> void override { is_busy_.store(false); }

    auto is_busy() const -> bool override { return is_busy_.load(); }

private:
    std::atomic<bool> is_busy_{false};
};

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_MOTION_HPP
