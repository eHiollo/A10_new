#include "a10_vr_vel_plan.hpp"

#include "kaanh/general/macro.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <rtb.hpp>

#include "a10_gripper_bridge.hpp"
#include "a10_policy_tcp_plan.hpp"
#include "a10_tcp_server.hpp"
#include "a10_vr_plan.hpp"

extern A10TcpServer* g_tcp_server;

namespace a10_tcp
{
namespace
{
constexpr int k_joint_num = 6;
constexpr int k_motor_base = 0;
constexpr double k_dt = 0.002;
constexpr double k_deg2rad = rtb::math::DEG2RAD;
constexpr double k_trans_dz = 0.001;
constexpr double k_rot_dz_rad = 0.2 * k_deg2rad;
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

void clamp_vec3(double* v, double max_norm)
{
    if (max_norm <= 0.0)
    {
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = 0.0;
        return;
    }
    const double n = vec3_norm(v);
    if (n > max_norm)
    {
        const double s = max_norm / n;
        v[0] *= s;
        v[1] *= s;
        v[2] *= s;
    }
}

void slew_vec3(double* v, const double* v_target, double max_dv)
{
    for (int i = 0; i < 3; ++i)
    {
        const double dv = std::clamp(v_target[i] - v[i], -max_dv, max_dv);
        v[i] += dv;
    }
}

void pm_to_rot3(const double* pm, double* R)
{
    R[0] = pm[0];
    R[1] = pm[1];
    R[2] = pm[2];
    R[3] = pm[4];
    R[4] = pm[5];
    R[5] = pm[6];
    R[6] = pm[8];
    R[7] = pm[9];
    R[8] = pm[10];
}

void mat3_transpose(const double* A, double* At)
{
    At[0] = A[0];
    At[1] = A[3];
    At[2] = A[6];
    At[3] = A[1];
    At[4] = A[4];
    At[5] = A[7];
    At[6] = A[2];
    At[7] = A[5];
    At[8] = A[8];
}

void mat3_mul(const double* A, const double* B, double* C)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            C[i * 3 + j] =
                A[i * 3 + 0] * B[0 * 3 + j] + A[i * 3 + 1] * B[1 * 3 + j] + A[i * 3 + 2] * B[2 * 3 + j];
        }
    }
}

void rotmat_to_rotvec(const double* R, double* w)
{
    const double tr = R[0] + R[4] + R[8];
    const double c = std::clamp(0.5 * (tr - 1.0), -1.0, 1.0);
    const double theta = std::acos(c);
    if (theta < rtb::math::EPSILON)
    {
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 0.0;
        return;
    }
    const double s = 2.0 * std::sin(theta);
    if (std::abs(s) < rtb::math::EPSILON)
    {
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 0.0;
        return;
    }
    w[0] = (R[7] - R[5]) / s * theta;
    w[1] = (R[2] - R[6]) / s * theta;
    w[2] = (R[3] - R[1]) / s * theta;
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

void pose_error_tool(
    const double* pm_tgt,
    const double* pm_act,
    double e_pos_tool[3],
    double e_rot_tool[3])
{
    double e_pos_base[3] = {
        pm_tgt[3] - pm_act[3],
        pm_tgt[7] - pm_act[7],
        pm_tgt[11] - pm_act[11],
    };
    aris::dynamic::s_inv_pm_dot_v3(const_cast<double*>(pm_tgt), e_pos_base, e_pos_tool);

    double R_tgt[9]{};
    double R_act[9]{};
    double R_act_T[9]{};
    double R_err[9]{};
    pm_to_rot3(pm_tgt, R_tgt);
    pm_to_rot3(pm_act, R_act);
    mat3_transpose(R_act, R_act_T);
    mat3_mul(R_tgt, R_act_T, R_err);

    double w_err_base[3]{};
    rotmat_to_rotvec(R_err, w_err_base);
    aris::dynamic::s_inv_pm_dot_v3(const_cast<double*>(pm_tgt), w_err_base, e_rot_tool);
}

void integrate_tool_twist(double* pm, const double v_tool[3], const double w_tool[3], double dt)
{
    double w_inc[3] = {w_tool[0] * dt, w_tool[1] * dt, w_tool[2] * dt};
    double d_tool[3] = {v_tool[0] * dt, v_tool[1] * dt, v_tool[2] * dt};

    double R_delta[9]{};
    rotvec_to_rm(w_inc, R_delta);

    double delta_pm[16]{};
    rtb::math::compose_transform(R_delta, d_tool, delta_pm);

    double pm_new[16]{};
    aris::dynamic::s_pm_dot_pm(pm, delta_pm, pm_new);
    std::memcpy(pm, pm_new, sizeof(pm_new));
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

void filter_rotvec_delta(double* w, double rot_gain)
{
    w[0] *= rot_gain;
    w[1] *= rot_gain;
    w[2] *= rot_gain;

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

void apply_vr_delta_to_target(double* target_pm, const std::vector<double>& delta, double rot_gain)
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
    filter_rotvec_delta(w_tool, rot_gain);

    double R_delta[9]{};
    rotvec_to_rm(w_tool, R_delta);

    double delta_pm[16]{};
    rtb::math::compose_transform(R_delta, d_tool, delta_pm);

    double target_new[16]{};
    aris::dynamic::s_pm_dot_pm(target_pm, delta_pm, target_new);
    std::memcpy(target_pm, target_new, sizeof(target_new));
}

bool is_effective_motion_delta(const std::vector<double>& delta, double rot_gain)
{
    if (delta.size() < 6)
    {
        return false;
    }
    const double k_rot_eps_rad = k_rot_dz_rad / std::max(rot_gain, 1e-9);
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(delta[static_cast<std::size_t>(i)]) >= k_trans_dz)
        {
            return true;
        }
    }
    const double w[3] = {delta[3], delta[4], delta[5]};
    return vec3_norm(w) >= k_rot_eps_rad;
}

void pull_target_toward_actual(double* target_pm, const double* actual_pm, double alpha)
{
    if (alpha <= rtb::math::EPSILON)
    {
        return;
    }
    double e_pos_tool[3]{};
    double e_rot_tool[3]{};
    pose_error_tool(target_pm, actual_pm, e_pos_tool, e_rot_tool);

    const double inv_dt = 1.0 / k_dt;
    const double v_pull[3] = {
        -e_pos_tool[0] * alpha * inv_dt,
        -e_pos_tool[1] * alpha * inv_dt,
        -e_pos_tool[2] * alpha * inv_dt,
    };
    const double w_pull[3] = {
        -e_rot_tool[0] * alpha * inv_dt,
        -e_rot_tool[1] * alpha * inv_dt,
        -e_rot_tool[2] * alpha * inv_dt,
    };
    integrate_tool_twist(target_pm, v_pull, w_pull, k_dt);
}

}  // namespace

struct A10VrVelDriver::Imp
{
    double target_pm[16]{};
    double command_pm[16]{};

    double v_cmd_tool[3]{};
    double w_cmd_tool[3]{};

    double current_joints[k_joint_num]{};
    double ik_joints[k_joint_num]{};
    double output_joints[k_joint_num]{};

    std::uint64_t consumed_ee_delta_seq_{0};
    std::int64_t last_packet_count_{0};
    bool inited_{false};
    bool idle_coast_{false};

    double max_lin_vel_{0.12};
    double max_ang_vel_{0.4};
    double max_lin_acc_{0.8};
    double max_ang_acc_{1.5};
    double target_pull_{0.995};
    double kp_pos_{3.0};
    double kp_rot_{2.0};
    double rot_gain_{1.0};
    double timeout_s_{0.12};

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

    void zero_motion_state()
    {
        v_cmd_tool[0] = 0.0;
        v_cmd_tool[1] = 0.0;
        v_cmd_tool[2] = 0.0;
        w_cmd_tool[0] = 0.0;
        w_cmd_tool[1] = 0.0;
        w_cmd_tool[2] = 0.0;
    }
};

auto A10VrVelDriver::prepareNrt() -> void
{
    imp_->inited_ = false;
    imp_->idle_coast_ = false;
    imp_->consumed_ee_delta_seq_ = 0;
    imp_->last_packet_count_ = 0;
    imp_->zero_motion_state();
    g_a10_vr_stop_requested.store(false, std::memory_order_release);

    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
    clear_vr_grip_cmd();
}

auto A10VrVelDriver::executeRT() -> int
{
    if (g_a10_policy_tcp_stop_requested.exchange(false, std::memory_order_acq_rel))
    {
        g_a10_vr_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        imp_->zero_motion_state();
        clear_vr_grip_cmd();
        mout() << "vr_vel: stop by policy stop flag" << std::endl;
        return 0;
    }

    if (g_a10_vr_stop_requested.load(std::memory_order_acquire))
    {
        g_a10_vr_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        imp_->zero_motion_state();
        clear_vr_grip_cmd();
        mout() << "vr_vel: stop by teleop stop flag" << std::endl;
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
            mout() << "vr_vel: need " << k_joint_num << " motors, have " << motors.size() << std::endl;
        }
        return 0;
    }
    auto& arm = arm_model(*this);
    auto& ee = ee_motion(*this);

    if (count() == 1)
    {
        if (doubleParam("vmax") > 1e-9) imp_->max_lin_vel_ = doubleParam("vmax");
        if (doubleParam("wmax") > 1e-9) imp_->max_ang_vel_ = doubleParam("wmax");
        if (doubleParam("amax") > 1e-9) imp_->max_lin_acc_ = doubleParam("amax");
        if (doubleParam("jacc") > 1e-9) imp_->max_ang_acc_ = doubleParam("jacc");
        if (doubleParam("pull") > 0.0 && doubleParam("pull") < 1.0)
        {
            imp_->target_pull_ = doubleParam("pull");
        }
        if (doubleParam("kp") >= 0.0) imp_->kp_pos_ = doubleParam("kp");
        if (doubleParam("kp_rot") >= 0.0) imp_->kp_rot_ = doubleParam("kp_rot");
        if (doubleParam("rot_gain") > 1e-9) imp_->rot_gain_ = doubleParam("rot_gain");
        if (doubleParam("timeout") > 1e-9) imp_->timeout_s_ = doubleParam("timeout");
        if (doubleParam("grip_vel") > 1e-9)
        {
            g_vr_grip_vel_mm_s.store(doubleParam("grip_vel"), std::memory_order_release);
        }
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
        std::memcpy(imp_->target_pm, T_base_to_ee, sizeof(imp_->target_pm));
        std::memcpy(imp_->command_pm, T_base_to_ee, sizeof(imp_->command_pm));
        std::memcpy(imp_->output_joints, imp_->current_joints, sizeof(imp_->output_joints));
        std::memcpy(imp_->ik_joints, imp_->current_joints, sizeof(imp_->ik_joints));
        imp_->zero_motion_state();
        imp_->last_packet_count_ = count();
        sync_vr_grip_target_from_actual(g_vr_grip_actual_mm.load(std::memory_order_acquire));
        imp_->inited_ = true;
        mout() << "vr_vel: init ok (P velocity, target+=delta, tool-frame)" << std::endl;
        return 1;
    }

    const double elapsed_since_packet =
        static_cast<double>(count() - imp_->last_packet_count_) * k_dt;

    std::vector<double> delta;
    std::uint64_t seq = 0;
    if (g_tcp_server->fetch_ee_delta_if_updated(delta, seq, imp_->consumed_ee_delta_seq_))
    {
        imp_->consumed_ee_delta_seq_ = seq;
        imp_->last_packet_count_ = count();
        if (delta.size() >= 7)
        {
            set_vr_grip_cmd(delta[6]);
        }
        if (is_effective_motion_delta(delta, imp_->rot_gain_))
        {
            imp_->idle_coast_ = false;
            apply_vr_delta_to_target(imp_->target_pm, delta, imp_->rot_gain_);
        }
        else
        {
            imp_->idle_coast_ = true;
        }
    }
    else if (elapsed_since_packet > imp_->timeout_s_)
    {
        imp_->idle_coast_ = true;
    }

    if (imp_->idle_coast_)
    {
        const double alpha = 1.0 - imp_->target_pull_;
        pull_target_toward_actual(imp_->target_pm, T_base_to_ee, alpha);
    }

    double e_pos_tool[3]{};
    double e_rot_tool[3]{};
    pose_error_tool(imp_->target_pm, T_base_to_ee, e_pos_tool, e_rot_tool);

    double v_target_tool[3] = {
        imp_->kp_pos_ * e_pos_tool[0],
        imp_->kp_pos_ * e_pos_tool[1],
        imp_->kp_pos_ * e_pos_tool[2],
    };
    double w_target_tool[3] = {
        imp_->kp_rot_ * e_rot_tool[0],
        imp_->kp_rot_ * e_rot_tool[1],
        imp_->kp_rot_ * e_rot_tool[2],
    };
    clamp_vec3(v_target_tool, imp_->max_lin_vel_);
    clamp_vec3(w_target_tool, imp_->max_ang_vel_);

    slew_vec3(imp_->v_cmd_tool, v_target_tool, imp_->max_lin_acc_ * k_dt);
    slew_vec3(imp_->w_cmd_tool, w_target_tool, imp_->max_ang_acc_ * k_dt);

    integrate_tool_twist(imp_->command_pm, imp_->v_cmd_tool, imp_->w_cmd_tool, k_dt);

    if (count() % 250 == 0)
    {
        mout() << "vr_vel diag: |e_pos|=" << vec3_norm(e_pos_tool) << "m |e_rot|=" << vec3_norm(e_rot_tool)
               << "rad |v|=" << vec3_norm(imp_->v_cmd_tool) << " |w|=" << vec3_norm(imp_->w_cmd_tool)
               << (imp_->idle_coast_ ? " idle" : "") << std::endl;
    }

    arm.setInputPos(imp_->output_joints);
    ee.setMpm(imp_->command_pm);
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
        mout() << "vr_vel: IK fail, |e_pos|=" << vec3_norm(e_pos_tool)
               << "m |e_rot|=" << vec3_norm(e_rot_tool) << "rad" << std::endl;
    }

    imp_->apply_joints_to_motors(*this);
    return count();
}

A10VrVelDriver::A10VrVelDriver(const std::string& name) : imp_(new Imp)
{
    (void)name;
    aris::core::fromXmlString(
        command(),
        "<Command name=\"vr_vel\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"kp\" abbreviation=\"p\" default=\"3.0\"/>"
        "    <Param name=\"kp_rot\" abbreviation=\"r\" default=\"2.0\"/>"
        "    <Param name=\"vmax\" abbreviation=\"v\" default=\"0.12\"/>"
        "    <Param name=\"wmax\" abbreviation=\"w\" default=\"0.4\"/>"
        "    <Param name=\"amax\" abbreviation=\"a\" default=\"0.8\"/>"
        "    <Param name=\"jacc\" abbreviation=\"j\" default=\"1.5\"/>"
        "    <Param name=\"pull\" abbreviation=\"l\" default=\"0.995\"/>"
        "    <Param name=\"rot_gain\" abbreviation=\"g\" default=\"1.0\"/>"
        "    <Param name=\"timeout\" abbreviation=\"t\" default=\"0.12\"/>"
        "    <Param name=\"grip_vel\" abbreviation=\"h\" default=\"50\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10VrVelDriver::~A10VrVelDriver() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10VrVelDriver)
}  // namespace a10_tcp

ARIS_REGISTRATION
{
    aris::core::class_<a10_tcp::A10VrVelDriver>("A10VrVelDriver").inherit<aris::plan::Plan>();
}
