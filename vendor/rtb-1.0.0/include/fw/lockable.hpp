#ifndef RTB_FRAMEWORK_LOCKABLE_HPP
#define RTB_FRAMEWORK_LOCKABLE_HPP

namespace rtb::fw {

/**
 * @brief 可锁定资源接口，用于 spawn 时避免多任务争抢同一硬件
 * MotionBase、ActuatorBase 等需独占控制的资源可实现此接口
 */
class LockableResource {
public:
    virtual ~LockableResource() = default;

    virtual auto try_lock() -> bool { return true; };

    virtual auto unlock() -> void {};

    virtual auto is_busy() const -> bool { return false; };
};

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_LOCKABLE_HPP
