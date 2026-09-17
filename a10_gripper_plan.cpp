#include "a10_gripper_plan.hpp"

#include "a10_gripper_bridge.hpp"
#include "gripper.hpp"

#include <iostream>

extern BusServo* g_gripper;

namespace a10_tcp
{
namespace
{
void skip_rt(aris::plan::Plan& plan)
{
    for (auto& m : plan.motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
    plan.option() |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
}

bool require_gripper(aris::plan::Plan& plan, const char* cmd)
{
    if (g_gripper == nullptr)
    {
        plan.mout() << cmd << ": gripper not connected (startup 应有 Gripper: connected)" << std::endl;
        return false;
    }
    return true;
}
}  // namespace

auto A10GripperOpen::prepareNrt() -> void
{
    skip_rt(*this);
    if (!require_gripper(*this, "g_open")) return;
    request_gripper_position_mm(k_gripper_mm_max);
    mout() << "g_open: request " << k_gripper_mm_max << " mm" << std::endl;
}

auto A10GripperOpen::executeRT() -> int { return 0; }

A10GripperOpen::A10GripperOpen(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"g_open\"/>");
}

A10GripperOpen::~A10GripperOpen() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10GripperOpen)

auto A10GripperClose::prepareNrt() -> void
{
    skip_rt(*this);
    if (!require_gripper(*this, "g_close")) return;
    request_gripper_position_mm(k_gripper_mm_min);
    mout() << "g_close: request " << k_gripper_mm_min << " mm" << std::endl;
}

auto A10GripperClose::executeRT() -> int { return 0; }

A10GripperClose::A10GripperClose(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"g_close\"/>");
}

A10GripperClose::~A10GripperClose() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10GripperClose)

auto A10GripperMove::prepareNrt() -> void
{
    skip_rt(*this);
    if (!require_gripper(*this, "g_move")) return;
    const double mm = doubleParam("pos");
    request_gripper_position_mm(mm);
    mout() << "g_move: request " << mm << " mm" << std::endl;
}

auto A10GripperMove::executeRT() -> int { return 0; }

A10GripperMove::A10GripperMove(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(
        command(),
        "<Command name=\"g_move\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"pos\" abbreviation=\"p\" default=\"50\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10GripperMove::~A10GripperMove() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10GripperMove)

auto A10GripperStatus::prepareNrt() -> void
{
    skip_rt(*this);
    mout() << "g_status: usb=" << (g_gripper ? "connected" : "not connected")
           << " target_mm=" << g_vr_grip_target_mm.load(std::memory_order_acquire)
           << " actual_mm=" << g_vr_grip_actual_mm.load(std::memory_order_acquire)
           << " vr_cmd=" << g_vr_grip_cmd.load(std::memory_order_acquire) << std::endl;
}

auto A10GripperStatus::executeRT() -> int { return 0; }

A10GripperStatus::A10GripperStatus(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"g_status\"/>");
}

A10GripperStatus::~A10GripperStatus() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10GripperStatus)
}  // namespace a10_tcp

ARIS_REGISTRATION
{
    aris::core::class_<a10_tcp::A10GripperOpen>("A10GripperOpen").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10GripperClose>("A10GripperClose").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10GripperMove>("A10GripperMove").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10GripperStatus>("A10GripperStatus").inherit<aris::plan::Plan>();
}
