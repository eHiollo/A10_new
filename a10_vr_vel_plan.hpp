#ifndef A10_VR_VEL_PLAN_HPP_
#define A10_VR_VEL_PLAN_HPP_

#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
/// VR P 速度遥操作：默认 ``SET_EE_DELTA``；A2.3 可显式切换为锚点 reference governor。
/// 两种模式复用同一套 ``target → P/限速/slew → command → IK`` 控制链。
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
