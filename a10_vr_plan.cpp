#include "a10_vr_plan.hpp"

#include "kaanh/general/macro.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <utility>
#include <vector>

#include <rtb.hpp>

#include "a10_gripper_bridge.hpp"
#include "a10_policy_tcp_plan.hpp"
#include "a10_tcp_server.hpp"

extern A10TcpServer* g_tcp_server;

namespace a10_tcp
{
std::atomic<bool> g_a10_vr_stop_requested{false};
std::atomic<bool> g_a10_vr_init_requested{false};

void request_vr_teleop_stop()
{
    g_a10_vr_stop_requested.store(true, std::memory_order_release);
    g_a10_vr_init_requested.store(false, std::memory_order_release);
    clear_vr_grip_cmd();
    if (g_tcp_server != nullptr)
    {
        g_tcp_server->clear_ee_delta_target_nrt();
    }
}

void request_vr_init()
{
    g_a10_vr_init_requested.store(true, std::memory_order_release);
    clear_vr_grip_cmd();
    if (g_tcp_server != nullptr)
    {
        g_tcp_server->clear_ee_delta_target_nrt();
    }
}

A10VrResetModule::A10VrResetModule()
{
    setName("VrReset");
}

auto A10VrResetModule::execute(
    const std::string& str, std::function<void(std::string)> send_ret) noexcept
    -> std::pair<std::string, std::string>
{
    if (getCmdName(str) != "reset")
    {
        return {str, ""};
    }
    request_vr_init();
    sendRet(send_ret, 0, "reset requested");
    std::cout << "reset: requested, vr_vel RT will home" << std::endl;
    END_CMD_FLOW;
}

namespace
{
constexpr int k_joint_num = 6;
constexpr int k_motor_base = 0;
constexpr double k_dt = 0.002;
constexpr double k_deg2rad = rtb::math::DEG2RAD;
constexpr double k_zero_va[6]{};

// VR TCP 30Hz：单包 delta 约为 15Hz 的一半，死区同比缩小
constexpr double k_trans_dz = 0.001;
constexpr double k_rot_dz_rad = 0.2 * k_deg2rad;
constexpr double k_rot_gain = 0.8;
constexpr double k_max_trans_step = 0.03;
constexpr double k_max_rot_step_rad = 1.8 * k_deg2rad;

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

double vec3_norm(const double* v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void rotvec_to_rm(const double* w, double* R_out)
{
    const double theta = vec3_norm(w);
    if (theta < rtb::math::EPSILON)
    {
        rtb::math::mat3_identity(R_out);
        return;
    }
    const double axis[3] = {w[0] / theta, w[1] / theta, w[2] / theta};
    rtb::math::rodrigues(axis, theta, R_out);
}

void filter_trans_delta(double* d)
{
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(d[i]) < k_trans_dz)
        {
            d[i] = 0.0;
        }
        if (d[i] > k_max_trans_step)
        {
            d[i] = k_max_trans_step;
        }
        if (d[i] < -k_max_trans_step)
        {
            d[i] = -k_max_trans_step;
        }
    }
}

void filter_rotvec_delta(double* w)
{
    w[0] *= k_rot_gain;
    w[1] *= k_rot_gain;
    w[2] *= k_rot_gain;

    const double theta = vec3_norm(w);
    if (theta < k_rot_dz_rad)
    {
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 0.0;
        return;
    }
    if (theta > k_max_rot_step_rad)
    {
        const double s = k_max_rot_step_rad / theta;
        w[0] *= s;
        w[1] *= s;
        w[2] *= s;
    }
}

void apply_vr_delta_rotvec_from_target(double* target_pm, const std::vector<double>& delta)
{
    double d_tool[3]{};
    double w_tool[3]{};
    for (int i = 0; i < 3 && i < static_cast<int>(delta.size()); ++i)
    {
        d_tool[i] = delta[static_cast<std::size_t>(i)];
    }
    for (int i = 3; i < 6 && i < static_cast<int>(delta.size()); ++i)
    {
        w_tool[i - 3] = delta[static_cast<std::size_t>(i)];
    }
    filter_trans_delta(d_tool);
    filter_rotvec_delta(w_tool);

    double R_delta[9]{};
    rotvec_to_rm(w_tool, R_delta);

    double delta_pm[16]{};
    rtb::math::compose_transform(R_delta, d_tool, delta_pm);

    double target_new[16]{};
    aris::dynamic::s_pm_dot_pm(target_pm, delta_pm, target_new);
    std::memcpy(target_pm, target_new, sizeof(target_new));
}

bool is_effective_delta_rotvec(const std::vector<double>& delta)
{
    if (delta.size() < 6)
    {
        return false;
    }
    constexpr double k_rot_eps_rad = k_rot_dz_rad / k_rot_gain;
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(delta[static_cast<std::size_t>(i)]) >= k_trans_dz)
        {
            return true;
        }
    }
    double w[3] = {
        delta[3],
        delta[4],
        delta[5],
    };
    return vec3_norm(w) >= k_rot_eps_rad;
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

struct A10VrDriver::Imp
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

    double max_lin_vel_{0.15};
    double max_lin_acc_{0.5};
    double max_ang_vel_{0.3};
    double max_ang_acc_{1.0};

    void apply_joints_to_motors(aris::plan::Plan& plan)
    {
        auto& motors = plan.controller()->motorPool();
        const aris::Size n_motors = motors.size();
        for (int i = 0; i < k_joint_num; ++i)
        {
            const int mi = k_motor_base + i;
            if (static_cast<aris::Size>(mi) >= n_motors)
            {
                break;
            }
            motors[mi].setTargetPos(output_joints[i]);
        }
    }
};

auto A10VrDriver::prepareNrt() -> void
{
    imp_->inited_ = false;
    imp_->consumed_ee_delta_seq_ = 0;
    g_a10_vr_stop_requested.store(false, std::memory_order_release);

    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }

    imp_->follower.setDt(k_dt);
    imp_->follower.setMaxVel(imp_->max_lin_vel_);
    imp_->follower.setMaxAcc(imp_->max_lin_acc_);
    imp_->follower.setMaxAngVel(imp_->max_ang_vel_);
    imp_->follower.setMaxAngAcc(imp_->max_ang_acc_);
    clear_vr_grip_cmd();
}

auto A10VrDriver::executeRT() -> int
{
    if (g_a10_policy_tcp_stop_requested.exchange(false, std::memory_order_acq_rel))
    {
        g_a10_vr_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        clear_vr_grip_cmd();
        mout() << "vr: stop by policy stop flag" << std::endl;
        return 0;
    }

    if (g_a10_vr_stop_requested.load(std::memory_order_acquire))
    {
        g_a10_vr_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        clear_vr_grip_cmd();
        mout() << "vr: stop by teleop stop flag" << std::endl;
        return 0;
    }

    if (g_tcp_server == nullptr)
    {
        return 0;
    }

    auto& motors = controller()->motorPool();
    if (motors.size() < static_cast<aris::Size>(k_joint_num))
    {
        if (count() == 1)
        {
            mout() << "vr: need " << k_joint_num << " motors, have " << motors.size() << std::endl;
        }
        return 0;
    }
    auto& arm = arm_model(*this);
    auto& ee = ee_motion(*this);

    if (count() == 1)
    {
        if (doubleParam("vel") > 1e-9) imp_->max_lin_vel_ = doubleParam("vel");
        if (doubleParam("acc") > 1e-9) imp_->max_lin_acc_ = doubleParam("acc");
        if (doubleParam("jvel") > 1e-9) imp_->max_ang_vel_ = doubleParam("jvel");
        if (doubleParam("jacc") > 1e-9) imp_->max_ang_acc_ = doubleParam("jacc");
        if (doubleParam("grip_vel") > 1e-9)
        {
            g_vr_grip_vel_mm_s.store(doubleParam("grip_vel"), std::memory_order_release);
        }

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
        sync_vr_grip_target_from_actual(g_vr_grip_actual_mm.load(std::memory_order_acquire));
        imp_->inited_ = true;
        mout() << "vr: init ok (tool-frame rotvec protocol)" << std::endl;
        return 1;
    }

    std::vector<double> delta;
    std::uint64_t seq = 0;
    if (g_tcp_server->fetch_ee_delta_if_updated(delta, seq, imp_->consumed_ee_delta_seq_))
    {
        imp_->consumed_ee_delta_seq_ = seq;
        if (delta.size() >= 7)
        {
            set_vr_grip_cmd(delta[6]);
        }
        if (is_effective_delta_rotvec(delta))
        {
            apply_vr_delta_rotvec_from_target(imp_->target_pm, delta);
            imp_->follower.setTargetPm(imp_->target_pm);
            imp_->follower.setTargetVa(k_zero_va);
        }
    }

    imp_->follower.moveSmoothDtAndGetResult(imp_->next_T_base_to_ee, imp_->next_vel, imp_->next_acc);

    if (count() % 250 == 0)
    {
        double pos_err = 0.0;
        double rot_err = 0.0;
        pm_pose_error(imp_->target_pm, T_base_to_ee, pos_err, rot_err);
        mout() << "vr diag: target-current pos_err=" << pos_err << "m rot_err=" << rot_err << "rad"
               << std::endl;
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
    else if (count() % 500 == 0)
    {
        double pos_err = 0.0;
        double rot_err = 0.0;
        pm_pose_error(imp_->next_T_base_to_ee, T_base_to_ee, pos_err, rot_err);
        mout() << "vr: IK fail, follow-current pos_err=" << pos_err << "m rot_err=" << rot_err << "rad" << std::endl;
    }

    imp_->apply_joints_to_motors(*this);
    return count();
}

A10VrDriver::A10VrDriver(const std::string& name) : imp_(new Imp)
{
    (void)name;
    aris::core::fromXmlString(
        command(),
        "<Command name=\"vr\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"vel\" abbreviation=\"v\" default=\"0.15\"/>"
        "    <Param name=\"acc\" abbreviation=\"a\" default=\"0.5\"/>"
        "    <Param name=\"jvel\" abbreviation=\"w\" default=\"0.3\"/>"
        "    <Param name=\"jacc\" abbreviation=\"b\" default=\"1.0\"/>"
        "    <Param name=\"grip_vel\" abbreviation=\"g\" default=\"60\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10VrDriver::~A10VrDriver() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10VrDriver)

auto A10VrCliStop::prepareNrt() -> void
{
    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
    request_vr_teleop_stop();
    option() |= NOT_RUN_EXECUTE_FUNCTION;
}

auto A10VrCliStop::executeRT() -> int
{
    return 0;
}

A10VrCliStop::A10VrCliStop(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"stop_teleop\"/>");
}

A10VrCliStop::~A10VrCliStop() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10VrCliStop)

auto A10VrCliInit::prepareNrt() -> void
{
    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
    request_vr_init();
    mout() << "reset: requested" << std::endl;
    option() |= NOT_RUN_EXECUTE_FUNCTION;
}

auto A10VrCliInit::executeRT() -> int
{
    return 0;
}

A10VrCliInit::A10VrCliInit(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"reset\"/>");
}

A10VrCliInit::~A10VrCliInit() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10VrCliInit)
}  // namespace a10_tcp

ARIS_REGISTRATION
{
    aris::core::class_<a10_tcp::A10VrDriver>("A10VrDriver").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10VrCliStop>("A10VrCliStop").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10VrCliInit>("A10VrCliInit").inherit<aris::plan::Plan>();
}
