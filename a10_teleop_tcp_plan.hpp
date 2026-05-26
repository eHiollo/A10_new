#ifndef A10_TELEOP_TCP_PLAN_HPP_
#define A10_TELEOP_TCP_PLAN_HPP_

#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
void request_vr_teleop_stop();

/// VR 遥操作：``SET_EE_DELTA`` → ``SE3Follower`` → IK → ``SingleAxisFollower`` 轴空间规划 → motor 0–5。
class A10TeleopTcpDriver : public aris::core::CloneObject<A10TeleopTcpDriver, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10TeleopTcpDriver();
    explicit A10TeleopTcpDriver(const std::string& name = "A10TeleopTcpDriver");

    KAANH_DECLARE_BIG_FOUR(A10TeleopTcpDriver)

private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;
};

class A10TeleopTcpCliStop : public aris::core::CloneObject<A10TeleopTcpCliStop, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10TeleopTcpCliStop();
    explicit A10TeleopTcpCliStop(const std::string& name = "A10TeleopTcpCliStop");

    KAANH_DECLARE_BIG_FOUR(A10TeleopTcpCliStop)
};
}  // namespace a10_tcp

#endif
