#ifndef A10_VR_PLAN_HPP_
#define A10_VR_PLAN_HPP_

#include <atomic>
#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
extern std::atomic<bool> g_a10_vr_stop_requested;
void request_vr_teleop_stop();

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
}  // namespace a10_tcp

#endif
