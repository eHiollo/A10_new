#ifndef A10_POLICY_TCP_PLAN_HPP_
#define A10_POLICY_TCP_PLAN_HPP_

#include <string>

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

namespace a10_tcp
{
/// 实时周期内根据 ``g_tcp_server`` 的 ``SET_JOINTS_BATCH`` / ``target_q_`` 驱动从臂关节（带速度插值）。
/// batch 队首与当前实际关节差超过 ``max_axis_delta`` / ``max_grip_delta`` 时丢弃该 keyframe。
/// 在 ``kaanh.xml`` 中增加对应 ``<A10PolicyTcpDriver .../>`` 后，终端执行 ``policy`` 启动；另注册 ``<A10PolicyTcpCliStop/>`` 后可用 ``stop`` 结束该驱动。
class A10PolicyTcpDriver : public aris::core::CloneObject<A10PolicyTcpDriver, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10PolicyTcpDriver();
    explicit A10PolicyTcpDriver(const std::string& name = "A10PolicyTcpDriver");

    KAANH_DECLARE_BIG_FOUR(A10PolicyTcpDriver)

private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;
};

/// 单拍 RT：置位退出请求并清空 TCP 轨迹队列；与 ``A10PolicyTcpDriver`` 配合，使运行中的 ``policy`` 在下一拍 ``return 0`` 退出。
class A10PolicyTcpCliStop : public aris::core::CloneObject<A10PolicyTcpCliStop, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;

    virtual ~A10PolicyTcpCliStop();
    explicit A10PolicyTcpCliStop(const std::string& name = "A10PolicyTcpCliStop");

    KAANH_DECLARE_BIG_FOUR(A10PolicyTcpCliStop)
};
}  // namespace a10_tcp

#endif
