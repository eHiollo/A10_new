#ifndef A10_VR_VEL_PLAN_HPP_
#define A10_VR_VEL_PLAN_HPP_

#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
/// VR 速度遥操作：``SET_EE_DELTA`` → 目标 twist；500Hz 加速度限幅平滑前馈 → IK。
/// 手柄 idle（零 delta）或断流后目标速度指数衰减滑行，非急停。
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
