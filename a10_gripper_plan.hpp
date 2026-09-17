#ifndef A10_GRIPPER_PLAN_HPP_
#define A10_GRIPPER_PLAN_HPP_

#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
class A10GripperOpen : public aris::core::CloneObject<A10GripperOpen, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10GripperOpen();
    explicit A10GripperOpen(const std::string& name = "A10GripperOpen");
    KAANH_DECLARE_BIG_FOUR(A10GripperOpen)
};

class A10GripperClose : public aris::core::CloneObject<A10GripperClose, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10GripperClose();
    explicit A10GripperClose(const std::string& name = "A10GripperClose");
    KAANH_DECLARE_BIG_FOUR(A10GripperClose)
};

class A10GripperMove : public aris::core::CloneObject<A10GripperMove, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10GripperMove();
    explicit A10GripperMove(const std::string& name = "A10GripperMove");
    KAANH_DECLARE_BIG_FOUR(A10GripperMove)
};

class A10GripperStatus : public aris::core::CloneObject<A10GripperStatus, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10GripperStatus();
    explicit A10GripperStatus(const std::string& name = "A10GripperStatus");
    KAANH_DECLARE_BIG_FOUR(A10GripperStatus)
};
}  // namespace a10_tcp

#endif
