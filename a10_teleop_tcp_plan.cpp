#include "a10_teleop_tcp_plan.hpp"

#include "kaanh/general/macro.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

#include <rtb.hpp>

#include "a10_tcp_server.hpp"
#include "a10_policy_tcp_plan.hpp"

extern A10TcpServer* g_tcp_server;

namespace a10_tcp
{
// ---------------------------------------------------------------------------
// VR 遥操作 RT Plan（Shell 命令 ``vr``）
//
// 数据流（500Hz）：
//   TCP SET_EE_DELTA → target_pm（笛卡尔目标，在上一拍目标上累加）
//   → SE3Follower（平滑 follow 点）→ Aris IK → motor 0–5 setTargetPos
//
// 停止：终端 ``stop`` / ``stop_teleop`` 在 prepareNrt 置 atomic 标志，本 plan 下一拍 return 0。
// ---------------------------------------------------------------------------

namespace
{
constexpr int k_joint_num = 6;
constexpr int k_motor_base = 0;   // 从臂关节对应 motorPool()[0..5]
constexpr double k_dt = 0.002;    // 500Hz，须与控制器 RT 周期一致
constexpr double k_deg2rad = rtb::math::DEG2RAD;
constexpr double k_zero_va[6]{};  // 目标/跟随点速度、加速度置零

// VR delta 死区（is_effective 与 apply 共用，避免「判定有效但 apply 滤成 0」）
constexpr double k_trans_dz = 0.002;       // 2 mm
constexpr double k_rot_dz_deg = 0.4;     // apply 内对 d*rot_gain 的死区
constexpr double k_rot_gain = 0.8;
constexpr double k_max_trans_step = 0.03;  // 30 mm / 包
constexpr double k_max_rot_step_deg = 1.8;

/** 取第一条子模型（六轴臂）及其末端 GeneralMotion。 */
auto arm_model(aris::plan::Plan& p) -> aris::dynamic::Model&
{
    auto& dual = dynamic_cast<aris::dynamic::MultiModel&>(p.modelBase()[0]);
    return dynamic_cast<aris::dynamic::Model&>(dual.subModels().at(0));
}

auto ee_motion(aris::plan::Plan& p) -> aris::dynamic::GeneralMotion&
{
    return dynamic_cast<aris::dynamic::GeneralMotion&>(arm_model(p).generalMotionPool().at(0));
}

/** 将 raw 角度 wrap 到 ref 附近 ±π，避免 IK 分支跳变。 */
double wrap_near(double ref, double raw)
{
    double d = raw - ref;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return ref + d;
}

/**
 * 在「上一拍 target_pm」上叠加 VR 增量（target = target ⊕ delta）。
 * 故意不用当前实机末端 T_base_to_ee，避免编码器噪声/抖动污染目标轨迹。
 *
 * delta[0..2]：平移 m；delta[3..5]：旋转 deg（基座系）；delta[6] 夹爪暂忽略。
 */
void apply_vr_delta_from_target(double* target_pm, const std::vector<double>& delta)
{
    double pe[6]{};
    rtb::math::pm2pe(target_pm, pe);

    // 死区 / 限幅：抑制 TCP 15Hz 小包噪声；单步过大则钳位防跳变
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
        double d_deg = delta[static_cast<std::size_t>(i)] * k_rot_gain;
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

/**
 * 判断 TCP 包是否「非零输入」（手柄未按时客户端通常发全 0）。
 * 阈值与 apply 死区一致：低于死区的包直接跳过，不更新 target。
 */
bool is_effective_delta(const std::vector<double>& delta)
{
    if (delta.size() < 6)
    {
        return false;
    }
    constexpr double k_rot_eps_deg = k_rot_dz_deg / k_rot_gain;  // 与 apply 旋转死区等效
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(delta[static_cast<std::size_t>(i)]) >= k_trans_dz)
        {
            return true;
        }
    }
    for (int i = 3; i < 6; ++i)
    {
        if (std::abs(delta[static_cast<std::size_t>(i)]) >= k_rot_eps_deg)
        {
            return true;
        }
    }
    return false;
}

/** 诊断用：比较两个 4×4 位姿在 pe 空间的最大平移/旋转误差。 */
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

// 由 stop / stop_teleop 的 prepareNrt 置位；executeRT 每拍开头检查
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

    double target_pm[16]{};           // 笛卡尔目标（VR delta 累加于此）
    double next_T_base_to_ee[16]{};  // follower 本拍输出的 follow 点位姿
    double next_vel[6]{};
    double next_acc[6]{};

    double current_joints[k_joint_num]{};  // 本拍实测关节
    double ik_joints[k_joint_num]{};       // IK 解
    double output_joints[k_joint_num]{};   // 下发关节（亦作 IK 种子）
    std::uint64_t consumed_ee_delta_seq_{0}; // 已消费的 TCP ee_delta_seq，防重复读同一包
    bool inited_{false};

    // 可被 Shell ``vr --v=...`` 或 kaanh.xml 默认值覆盖
    double max_lin_vel_{0.06};
    double max_lin_acc_{0.5};
    double max_ang_vel_{0.3};
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
    // --- 停止检查（须在主逻辑之前）---
    // policy 的 stop 也会置 g_a10_policy_tcp_stop_requested，此处兼容统一 stop 命令
    if (g_a10_policy_tcp_stop_requested.exchange(false, std::memory_order_acq_rel))
    {
        g_a10_teleop_tcp_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        mout() << "vr: stop by policy stop flag" << std::endl;
        return 0;
    }

    if (g_a10_teleop_tcp_stop_requested.load(std::memory_order_acquire))
    {
        g_a10_teleop_tcp_stop_requested.store(false, std::memory_order_release);
        imp_->inited_ = false;
        mout() << "vr: stop by teleop stop flag" << std::endl;
        return 0;
    }

    if (g_tcp_server == nullptr)
    {
        return 0;
    }

    auto& motors = controller()->motorPool();
    auto& arm = arm_model(*this);
    auto& ee = ee_motion(*this);

    // 首拍应用 Shell/XML 运动学限制参数到 follower
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

    // --- 读实机状态 + 正解 ---
    for (int i = 0; i < k_joint_num; ++i)
    {
        imp_->current_joints[i] = motors[k_motor_base + i].actualPos();
    }

    double T_base_to_ee[16]{};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    arm.setInputPos(imp_->current_joints);
    if (arm.forwardKinematics())
    {
        return count();  // FK 失败则本拍跳过，保持 plan 运行
    }
    ee.updP();
    ee.getMpm(T_base_to_ee);

    // --- 首拍初始化：锚定 target/follow 到当前末端，不读 VR ---
    if (!imp_->inited_)
    {
        // 诊断：同一 T_base_to_ee 立刻 IK，验证模型/电机映射是否正常
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
        // 须先 reset 再 set，避免 reset 清掉刚写入的状态
        imp_->follower.reset();
        imp_->follower.setTargetPm(imp_->target_pm);
        imp_->follower.setTargetVa(k_zero_va);
        imp_->follower.setFollowPm(imp_->next_T_base_to_ee);
        imp_->follower.setFollowVa(k_zero_va);
        std::memcpy(imp_->output_joints, imp_->current_joints, sizeof(imp_->output_joints));
        std::memcpy(imp_->ik_joints, imp_->current_joints, sizeof(imp_->ik_joints));
        imp_->inited_ = true;
        mout() << "vr: init ok" << std::endl;
        return 1;  // 本拍只做初始化，下一拍进入主循环
    }

    // --- 读 VR 增量（仅 seq 更新时取最新一包）---
    std::vector<double> delta;
    std::uint64_t seq = 0;
    if (g_tcp_server->fetch_ee_delta_if_updated(delta, seq, imp_->consumed_ee_delta_seq_))
    {
        imp_->consumed_ee_delta_seq_ = seq;
        if (is_effective_delta(delta))
        {
            apply_vr_delta_from_target(imp_->target_pm, delta);
            imp_->follower.setTargetPm(imp_->target_pm);
            imp_->follower.setTargetVa(k_zero_va);
            // 只更新 target，不 reset follow；由 moveSmooth 从当前 follow 追向新 target
        }
    }

    // --- 笛卡尔平滑：输出 follow 点 next_T_base_to_ee（非直接 target）---
    imp_->follower.moveSmoothDtAndGetResult(imp_->next_T_base_to_ee, imp_->next_vel, imp_->next_acc);

    if (count() % 250 == 0)
    {
        double pos_err = 0.0;
        double rot_err = 0.0;
        pm_pose_error(imp_->target_pm, T_base_to_ee, pos_err, rot_err);
        mout() << "vr diag: target-current pos_err=" << pos_err << "m rot_err=" << rot_err << "rad"
               << std::endl;
    }

    // --- 逆解：follow 点 → 关节；失败则保持上一拍 output_joints ---
    arm.setInputPos(imp_->output_joints);  // 上一拍关节作 IK 种子，保证解的连续性
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
    return count();  // 非 0：持续 plan；return 0 退出
}

A10TeleopTcpDriver::A10TeleopTcpDriver(const std::string& name) : imp_(new Imp)
{
    (void)name;
    aris::core::fromXmlString(
        command(),
        "<Command name=\"vr\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"vel\" abbreviation=\"v\" default=\"0.06\"/>"
        "    <Param name=\"acc\" abbreviation=\"a\" default=\"0.5\"/>"
        "    <Param name=\"jvel\" abbreviation=\"w\" default=\"0.3\"/>"
        "    <Param name=\"jacc\" abbreviation=\"b\" default=\"1.0\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10TeleopTcpDriver::~A10TeleopTcpDriver() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10TeleopTcpDriver)

// stop_teleop：仅 prepareNrt 置标志，不占用 RT（vr 持续运行时 executeRT 排不上队）
auto A10TeleopTcpCliStop::prepareNrt() -> void
{
    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
    request_vr_teleop_stop();
    option() |= NOT_RUN_EXECUTE_FUNCTION;
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
