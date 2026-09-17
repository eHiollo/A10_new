#ifndef A10_VR_VEL_PLAN_HPP_
#define A10_VR_VEL_PLAN_HPP_

#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
/// VR P 速度遥操作：``SET_EE_DELTA`` 累加 ``target_pm``；``v = Kp×(target−actual)`` 限幅 + slew → IK。
/// idle 时 ``target`` 缓慢贴回实测位姿。夹爪同 ``vr``。
/// ``reset`` 不退出本驱动，内部关节回到初始位姿后重统一位姿跟踪。
class A10VrVelDriver : public aris::core::CloneObject<A10VrVelDriver, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10VrVelDriver();
    explicit A10VrVelDriver(const std::string& name = "A10VrVelDriver");

    KAANH_DECLARE_BIG_FOUR(A10VrVelDriver)

private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;
};
}  // namespace a10_tcp

#endif
