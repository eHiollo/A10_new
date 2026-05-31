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

/** 在「上一拍目标」上叠加 VR 增量，避免把实机抖动回灌到目标轨迹。 */
void apply_vr_delta_from_target(double* target_pm, const std::vector<double>& delta)
{
    double pe[6]{};
    rtb::math::pm2pe(target_pm, pe);

    constexpr double k_trans_dz = 0.001;
    constexpr double k_rot_dz_deg = 0.5;
    constexpr double k_max_trans_step = 0.015;
    constexpr double k_max_rot_step_deg = 3.0;

    for (int i = 0; i < 3 && i < static_cast<int>(delta.size()); ++i)
    {
        double d = delta[static_cast<std::size_t>(i)];
        if (std::abs(d) < k_trans_dz)
        {
            d = 0.0;
        }
        if (d > k_max_trans_step)
        {
            d = k_max_trans_step;
        }
        if (d < -k_max_trans_step)
        {
            d = -k_max_trans_step;
        }
        pe[i] += d;
    }
    for (int i = 3; i < 6 && i < static_cast<int>(delta.size()); ++i)
    {
        double d_deg = delta[static_cast<std::size_t>(i)];
        if (std::abs(d_deg) < k_rot_dz_deg)
        {
            d_deg = 0.0;
        }
        if (d_deg > k_max_rot_step_deg)
        {
            d_deg = k_max_rot_step_deg;
        }
        if (d_deg < -k_max_rot_step_deg)
        {
            d_deg = -k_max_rot_step_deg;
        }
        pe[i] = wrap_near(pe[i], pe[i] + d_deg * k_deg2rad);
    }
    rtb::math::pe2pm(pe, target_pm);
}

bool is_effective_delta(const std::vector<double>& delta)
{
    if (delta.size() < 6)
    {
        return false;
    }
    constexpr double k_pos_eps = 5e-5;       // 0.05 mm
    constexpr double k_rot_eps_deg = 0.05;   // 0.05 deg
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(delta[static_cast<std::size_t>(i)]) > k_pos_eps)
        {
            return true;
        }
    }
    for (int i = 3; i < 6; ++i)
    {
        if (std::abs(delta[static_cast<std::size_t>(i)]) > k_rot_eps_deg)
        {
            return true;
        }
    }
    return false;
}

void pm_pose_error(const double* pm_ref, const double* pm_now, double& pos_err, double& rot_err)
{
    double pe_ref[6]{};
    double pe_now[6]{};
    rtb::math::pm2pe(const_cast<double*>(pm_ref), pe_ref);
    rtb::math::pm2pe(const_cast<double*>(pm_now), pe_now);
    pos_err = 0.0;
    rot_err = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        pos_err = std::max(pos_err, std::abs(pe_ref[i] - pe_now[i]));
    }
    for (int i = 3; i < 6; ++i)
    {
        rot_err = std::max(rot_err, std::abs(wrap_near(pe_now[i], pe_ref[i]) - pe_now[i]));
    }
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
    rtb::plan::SE3Follower follower;

    double target_pm[16]{};
    double next_T_base_to_ee[16]{};
    double next_vel[6]{};
    double next_acc[6]{};

    double current_joints[k_joint_num]{};
    double ik_joints[k_joint_num]{};
    double output_joints[k_joint_num]{};
    std::uint64_t consumed_ee_delta_seq_{0};
    bool inited_{false};
    int no_effective_delta_ticks_{0};

    double max_lin_vel_{0.03};
    double max_lin_acc_{0.5};
    double max_ang_vel_{0.4};
    double max_ang_acc_{1.0};

    void apply_joints_to_motors(aris::plan::Plan& plan)
    {
        auto& motors = plan.controller()->motorPool();
        for (int i = 0; i < k_joint_num; ++i)
        {
            motors[k_motor_base + i].setTargetPos(output_joints[i]);
        }
    }
};

auto A10TeleopTcpDriver::prepareNrt() -> void
{
    imp_->inited_ = false;
    imp_->consumed_ee_delta_seq_ = 0;
    imp_->no_effective_delta_ticks_ = 0;
    g_a10_teleop_tcp_stop_requested.store(false, std::memory_order_release);

    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }

    imp_->follower.setDt(k_dt);
    imp_->follower.setMaxVel(imp_->max_lin_vel_);
    imp_->follower.setMaxAcc(imp_->max_lin_acc_);
    imp_->follower.setMaxAngVel(imp_->max_ang_vel_);
    imp_->follower.setMaxAngAcc(imp_->max_ang_acc_);
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

    auto& motors = controller()->motorPool();
    auto& arm = arm_model(*this);
    auto& ee = ee_motion(*this);

    if (count() == 1)
    {
        if (doubleParam("vel") > 1e-9) imp_->max_lin_vel_ = doubleParam("vel");
        if (doubleParam("acc") > 1e-9) imp_->max_lin_acc_ = doubleParam("acc");
        if (doubleParam("jvel") > 1e-9) imp_->max_ang_vel_ = doubleParam("jvel");
        if (doubleParam("jacc") > 1e-9) imp_->max_ang_acc_ = doubleParam("jacc");

        imp_->follower.setMaxVel(imp_->max_lin_vel_);
        imp_->follower.setMaxAcc(imp_->max_lin_acc_);
        imp_->follower.setMaxAngVel(imp_->max_ang_vel_);
        imp_->follower.setMaxAngAcc(imp_->max_ang_acc_);
    }

    for (int i = 0; i < k_joint_num; ++i)
    {
        imp_->current_joints[i] = motors[k_motor_base + i].actualPos();
    }

    double T_base_to_ee[16]{};
    arm.setInputPos(imp_->current_joints);
    if (arm.forwardKinematics())
    {
        return count();
    }
    ee.updP();
    ee.getMpm(T_base_to_ee);

    if (!imp_->inited_)
    {
        // 诊断 1: 当前关节 FK 后立刻 IK 的闭环自检（用于排查模型/映射问题）。
        {
            double q_check[k_joint_num]{};
            arm.setInputPos(imp_->current_joints);
            ee.setMpm(T_base_to_ee);
            if (!arm.inverseKinematics())
            {
                arm.getInputPos(q_check);
                double dq_max = 0.0;
                for (int i = 0; i < k_joint_num; ++i)
                {
                    dq_max = std::max(dq_max, std::abs(wrap_near(imp_->current_joints[i], q_check[i]) - imp_->current_joints[i]));
                }
                mout() << "vr diag: FK->IK self-check ok, max|dq|=" << dq_max << std::endl;
            }
            else
            {
                mout() << "vr diag: FK->IK self-check FAILED" << std::endl;
            }
        }

        std::memcpy(imp_->target_pm, T_base_to_ee, sizeof(imp_->target_pm));
        std::memcpy(imp_->next_T_base_to_ee, T_base_to_ee, sizeof(imp_->next_T_base_to_ee));
        imp_->follower.reset();
        imp_->follower.setTargetPm(imp_->target_pm);
        imp_->follower.setTargetVa(k_zero_va);
        imp_->follower.setFollowPm(imp_->next_T_base_to_ee);
        imp_->follower.setFollowVa(k_zero_va);
        std::memcpy(imp_->output_joints, imp_->current_joints, sizeof(imp_->output_joints));
        std::memcpy(imp_->ik_joints, imp_->current_joints, sizeof(imp_->ik_joints));
        imp_->inited_ = true;
        mout() << "vr: init ok" << std::endl;
        return 1;
    }

    std::vector<double> delta;
    std::uint64_t seq = 0;
    if (g_tcp_server->fetch_ee_delta_if_updated(delta, seq, imp_->consumed_ee_delta_seq_))
    {
        imp_->consumed_ee_delta_seq_ = seq;
        if (is_effective_delta(delta))
        {
            imp_->no_effective_delta_ticks_ = 0;
            apply_vr_delta_from_target(imp_->target_pm, delta);
            imp_->follower.setTargetPm(imp_->target_pm);
            imp_->follower.setTargetVa(k_zero_va);
        }
        else
        {
            ++imp_->no_effective_delta_ticks_;
        }
    }
    else
    {
        ++imp_->no_effective_delta_ticks_;
    }

    // 长时间零输入时直接“抱住当前位置”，避免 IK 在静止附近抖动。
    if (imp_->no_effective_delta_ticks_ > 50)
    {
        std::memcpy(imp_->target_pm, T_base_to_ee, sizeof(imp_->target_pm));
        std::memcpy(imp_->next_T_base_to_ee, T_base_to_ee, sizeof(imp_->next_T_base_to_ee));
        imp_->follower.setFollowPm(imp_->next_T_base_to_ee);
        imp_->follower.setFollowVa(k_zero_va);
        imp_->follower.setTargetPm(imp_->target_pm);
        imp_->follower.setTargetVa(k_zero_va);
    }

    imp_->follower.moveDtAndGetResult(imp_->next_T_base_to_ee, imp_->next_vel, imp_->next_acc);

    if (count() % 250 == 0)
    {
        double pos_err = 0.0;
        double rot_err = 0.0;
        pm_pose_error(imp_->target_pm, T_base_to_ee, pos_err, rot_err);
        mout() << "vr diag: target-current pos_err=" << pos_err << "m rot_err=" << rot_err << "rad"
               << " idle_ticks=" << imp_->no_effective_delta_ticks_ << std::endl;
    }

    arm.setInputPos(imp_->output_joints);
    ee.setMpm(imp_->next_T_base_to_ee);
    if (!arm.inverseKinematics())
    {
        arm.getInputPos(imp_->ik_joints);
        for (int i = 0; i < k_joint_num; ++i)
        {
            imp_->ik_joints[i] = wrap_near(imp_->output_joints[i], imp_->ik_joints[i]);
        }
        std::memcpy(imp_->output_joints, imp_->ik_joints, sizeof(imp_->output_joints));
    }
    else if (count() % 250 == 0)
    {
        double pos_err = 0.0;
        double rot_err = 0.0;
        pm_pose_error(imp_->next_T_base_to_ee, T_base_to_ee, pos_err, rot_err);
        mout() << "vr: IK fail, follow-current pos_err=" << pos_err << "m rot_err=" << rot_err << "rad" << std::endl;
    }

    imp_->apply_joints_to_motors(*this);
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
        "    <Param name=\"acc\" abbreviation=\"a\" default=\"0.5\"/>"
        "    <Param name=\"jvel\" abbreviation=\"w\" default=\"0.4\"/>"
        "    <Param name=\"jacc\" abbreviation=\"b\" default=\"1.0\"/>"
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
