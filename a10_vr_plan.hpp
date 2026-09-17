#ifndef A10_VR_PLAN_HPP_
#define A10_VR_PLAN_HPP_

#include <atomic>
#include <functional>
#include <string>
#include <utility>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"

namespace a10_tcp
{
extern std::atomic<bool> g_a10_vr_stop_requested;
extern std::atomic<bool> g_a10_vr_init_requested;
void request_vr_teleop_stop();
void request_vr_init();

/// Shell 拦截 ``reset``：不进 Plan 队列，只给运行中的 ``vr_vel`` 置位。
class A10VrResetModule : public kaanh::module::MiddleModule
{
public:
    auto execute(const std::string& str, std::function<void(std::string)> send_ret) noexcept
        -> std::pair<std::string, std::string> override;

    A10VrResetModule();
};

/// VR 遥操作（tool 系 rotvec）：``SET_EE_DELTA`` 7 维 ``[dx,dy,dz, rx,ry,rz, grip]``（tool 系 m/rad；
/// ``grip`` 为摇杆 [-1,1]，+1 张开）→ ``SE3Follower`` → IK；夹爪经 ``BusServo`` 非 RT 线程。
class A10VrDriver : public aris::core::CloneObject<A10VrDriver, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10VrDriver();
    explicit A10VrDriver(const std::string& name = "A10VrDriver");

    KAANH_DECLARE_BIG_FOUR(A10VrDriver)

private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;
};

class A10VrCliStop : public aris::core::CloneObject<A10VrCliStop, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10VrCliStop();
    explicit A10VrCliStop(const std::string& name = "A10VrCliStop");

    KAANH_DECLARE_BIG_FOUR(A10VrCliStop)
};

/// 不抢 RT：命令 ``reset`` 只置位，由运行中的 ``vr_vel`` 内部回到初始位姿。
class A10VrCliInit : public aris::core::CloneObject<A10VrCliInit, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10VrCliInit();
    explicit A10VrCliInit(const std::string& name = "A10VrCliInit");

    KAANH_DECLARE_BIG_FOUR(A10VrCliInit)
};
}  // namespace a10_tcp

#endif
