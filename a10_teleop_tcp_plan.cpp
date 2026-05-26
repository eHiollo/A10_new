#include "a10_teleop_tcp_plan.hpp"

#include "kaanh/general/macro.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

#include <rtb.hpp>

#include "a10_tcp_server.hpp"

extern A10TcpServer* g_tcp_server;

namespace a10_tcp
{
namespace
{
constexpr int k_aris_id = 0;
constexpr int k_joint_num = 6;
constexpr int k_motor_base = 0;
constexpr double k_dt = 0.002;
constexpr double k_deg2rad = rtb::math::DEG2RAD;
constexpr double k_zero_va[6]{};

auto arm_model(aris::plan::Plan& p) -> aris::dynamic::Model&
{
    auto& dual = dynamic_cast<aris::dynamic::MultiModel&>(p.modelBase()[0]);
    return dynamic_cast<aris::dynamic::Model&>(dual.subModels().at(0));
}

auto ee_motion(aris::plan::Plan& p) -> aris::dynamic::GeneralMotion&
{
    return dynamic_cast<aris::dynamic::GeneralMotion&>(arm_model(p).generalMotionPool().at(0));
}

double wrap_near(double ref, double raw)
{
    double d = raw - ref;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return ref + d;
}

void apply_vr_delta_to_pm(
    double* target_pm,
    const std::vector<double>& delta,
    double pos_scale,
    double rot_scale)
{
    double pe[6]{};
    rtb::math::pm2pe(target_pm, pe);
    for (int i = 0; i < 3 && i < static_cast<int>(delta.size()); ++i)
    {
        pe[i] += delta[static_cast<std::size_t>(i)] * pos_scale;
    }
    for (int i = 3; i < 6 && i < static_cast<int>(delta.size()); ++i)
    {
        pe[i] = wrap_near(pe[i], pe[i] + delta[static_cast<std::size_t>(i)] * k_deg2rad * rot_scale);
    }
    rtb::math::pe2pm(pe, target_pm);
}

}  // namespace

std::atomic<bool> g_a10_teleop_tcp_stop_requested{false};

void request_vr_teleop_stop()
{
    g_a10_teleop_tcp_stop_requested.store(true, std::memory_order_release);
    if (g_tcp_server != nullptr)
    {
        g_tcp_server->clear_ee_delta_target_nrt();
    }
}

struct A10TeleopTcpDriver::Imp
{
    rtb::core::RobotInterface ri;
    rtb::plan::SE3Follower follower;
    rtb::plan::SingleAxisFollower joint_followers[k_joint_num];

    double target_pm[16]{};
    double next_T_base_to_ee[16]{};
    double next_vel[6]{};
    double next_acc[6]{};

    double current_joints[k_joint_num]{};
    double ik_joints[k_joint_num]{};
    double output_joints[k_joint_num]{};
    std::uint64_t consumed_ee_delta_seq_{0};
    bool inited_{false};

    double max_lin_vel_{0.05};
    double max_lin_acc_{5.0};
    double max_ang_vel_{1.0};
    double max_ang_acc_{10.0};
    double max_joint_vel_{0.35};
    double max_joint_acc_{2.0};
    double pos_scale_{1.0};
    double rot_scale_{1.0};

    void setup_joint_followers()
    {
        for (int i = 0; i < k_joint_num; ++i)
        {
            joint_followers[i].setDt(k_dt);
            joint_followers[i].setMaxVel(max_joint_vel_);
            joint_followers[i].setMaxAcc(max_joint_acc_);
        }
    }

    void init_joint_followers_from_current()
    {
        for (int i = 0; i < k_joint_num; ++i)
        {
            joint_followers[i].setFollow(current_joints[i], 0.0);
            joint_followers[i].setTarget(current_joints[i], 0.0);
        }
        std::memcpy(output_joints, current_joints, sizeof(output_joints));
        std::memcpy(ik_joints, current_joints, sizeof(ik_joints));
    }

    void set_joint_targets_from_ik()
    {
        for (int i = 0; i < k_joint_num; ++i)
        {
            joint_followers[i].setTarget(wrap_near(output_joints[i], ik_joints[i]), 0.0);
        }
    }

    void track_joints_and_apply()
    {
        for (int i = 0; i < k_joint_num; ++i)
        {
            double pos = 0.0;
            double vel = 0.0;
            double acc = 0.0;
            joint_followers[i].moveDtAndGetResult(pos, vel, acc);
            output_joints[i] = pos;
        }
        ri.setJoints(k_aris_id, output_joints);
    }
};

auto A10TeleopTcpDriver::prepareNrt() -> void
{
    imp_->inited_ = false;
    imp_->consumed_ee_delta_seq_ = 0;
    g_a10_teleop_tcp_stop_requested.store(false, std::memory_order_release);

    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }

    imp_->follower.setDt(k_dt);
    imp_->setup_joint_followers();

    imp_->ri.setGetJointsFunc(k_aris_id, [this](double* q) {
        auto& motors = controller()->motorPool();
        for (int i = 0; i < k_joint_num; ++i)
        {
            q[i] = motors[k_motor_base + i].actualPos();
        }
    });

    imp_->ri.setSetJointsFunc(k_aris_id, [this](const double* q) {
        auto& motors = controller()->motorPool();
        for (int i = 0; i < k_joint_num; ++i)
        {
            motors[k_motor_base + i].setTargetPos(q[i]);
        }
    });

    imp_->ri.setForwardKinematicsFunc(k_aris_id, [this](const double* q, double* pm) -> bool {
        auto& arm = arm_model(*this);
        auto& ee = ee_motion(*this);
        arm.setInputPos(const_cast<double*>(q));
        if (arm.forwardKinematics())
        {
            return false;
        }
        ee.updP();
        ee.getMpm(pm);
        return true;
    });

    imp_->ri.setInverseKinematicsFunc(k_aris_id, [this](const double* pm, double* q) -> bool {
        auto& arm = arm_model(*this);
        auto& ee = ee_motion(*this);
        arm.setInputPos(imp_->output_joints);
        ee.setMpm(const_cast<double*>(pm));
        if (arm.inverseKinematics())
        {
            return false;
        }
        arm.getInputPos(q);
        for (int i = 0; i < k_joint_num; ++i)
        {
            q[i] = wrap_near(imp_->output_joints[i], q[i]);
        }
        return true;
    });
}

auto A10TeleopTcpDriver::executeRT() -> int
{
    if (g_a10_teleop_tcp_stop_requested.load(std::memory_order_acquire))
    {
        g_a10_teleop_tcp_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        mout() << "vr: stop" << std::endl;
        return 0;
    }

    if (g_tcp_server == nullptr)
    {
        return 0;
    }

    if (count() == 1)
    {
        imp_->max_lin_vel_ = doubleParam("vel");
        imp_->max_lin_acc_ = doubleParam("acc");
        imp_->max_ang_vel_ = doubleParam("jvel");
        imp_->max_ang_acc_ = doubleParam("jacc");
        if (doubleParam("pos_scale") > 1e-9)
        {
            imp_->pos_scale_ = doubleParam("pos_scale");
        }
        if (doubleParam("rot_scale") > 1e-9)
        {
            imp_->rot_scale_ = doubleParam("rot_scale");
        }
        imp_->follower.setMaxVel(imp_->max_lin_vel_);
        imp_->follower.setMaxAcc(imp_->max_lin_acc_);
        imp_->follower.setMaxAngVel(imp_->max_ang_vel_);
        imp_->follower.setMaxAngAcc(imp_->max_ang_acc_);
        imp_->max_joint_vel_ = doubleParam("qvel");
        imp_->max_joint_acc_ = doubleParam("qacc");
        imp_->setup_joint_followers();
    }

    imp_->ri.getJoints(k_aris_id, imp_->current_joints);

    double T_base_to_ee[16]{};
    if (!imp_->ri.forwardKinematics(k_aris_id, imp_->current_joints, T_base_to_ee))
    {
        return count();
    }

    if (!imp_->inited_)
    {
        std::memcpy(imp_->target_pm, T_base_to_ee, sizeof(imp_->target_pm));
        std::memcpy(imp_->next_T_base_to_ee, T_base_to_ee, sizeof(imp_->next_T_base_to_ee));
        imp_->follower.setTargetPm(imp_->target_pm);
        imp_->follower.setTargetVa(k_zero_va);
        imp_->follower.setFollowPm(imp_->next_T_base_to_ee);
        imp_->follower.setFollowVa(k_zero_va);
        imp_->follower.reset();
        imp_->init_joint_followers_from_current();
        imp_->inited_ = true;
        mout() << "vr: init ok" << std::endl;
        return 1;
    }

    std::vector<double> delta;
    std::uint64_t seq = 0;
    if (g_tcp_server->fetch_ee_delta_if_updated(delta, seq, imp_->consumed_ee_delta_seq_))
    {
        imp_->consumed_ee_delta_seq_ = seq;
        apply_vr_delta_to_pm(imp_->target_pm, delta, imp_->pos_scale_, imp_->rot_scale_);
        imp_->follower.setTargetPm(imp_->target_pm);
        imp_->follower.setTargetVa(k_zero_va);
    }

    imp_->follower.moveDtAndGetResult(imp_->next_T_base_to_ee, imp_->next_vel, imp_->next_acc);

    if (imp_->ri.inverseKinematics(k_aris_id, imp_->next_T_base_to_ee, imp_->ik_joints))
    {
        imp_->set_joint_targets_from_ik();
    }
    else if (count() % 250 == 0)
    {
        mout() << "vr: IK fail" << std::endl;
    }

    imp_->track_joints_and_apply();
    return count();
}

A10TeleopTcpDriver::A10TeleopTcpDriver(const std::string& name) : imp_(new Imp)
{
    (void)name;
    aris::core::fromXmlString(
        command(),
        "<Command name=\"vr\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"vel\" abbreviation=\"v\" default=\"0.03\"/>"
        "    <Param name=\"acc\" abbreviation=\"a\" default=\"3.0\"/>"
        "    <Param name=\"jvel\" abbreviation=\"w\" default=\"1.0\"/>"
        "    <Param name=\"jacc\" abbreviation=\"b\" default=\"5.0\"/>"
        "    <Param name=\"qvel\" default=\"0.35\"/>"
        "    <Param name=\"qacc\" default=\"2.0\"/>"
        "    <Param name=\"pos_scale\" abbreviation=\"s\" default=\"1.0\"/>"
        "    <Param name=\"rot_scale\" abbreviation=\"r\" default=\"1.0\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10TeleopTcpDriver::~A10TeleopTcpDriver() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10TeleopTcpDriver)

auto A10TeleopTcpCliStop::prepareNrt() -> void
{
    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
    request_vr_teleop_stop();
}

auto A10TeleopTcpCliStop::executeRT() -> int
{
    return 0;
}

A10TeleopTcpCliStop::A10TeleopTcpCliStop(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"stop_teleop\"/>");
}

A10TeleopTcpCliStop::~A10TeleopTcpCliStop() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10TeleopTcpCliStop)

}  // namespace a10_tcp

ARIS_REGISTRATION
{
    aris::core::class_<a10_tcp::A10TeleopTcpDriver>("A10TeleopTcpDriver").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10TeleopTcpCliStop>("A10TeleopTcpCliStop").inherit<aris::plan::Plan>();
}
