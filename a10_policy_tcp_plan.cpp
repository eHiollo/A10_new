#include "a10_policy_tcp_plan.hpp"

#include "kaanh/general/macro.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <string>
#include <vector>

#include "a10_tcp_server.hpp"
#include "a10_policy_smoother.hpp"
#include "a10_vr_plan.hpp"
#include "a10_gripper_bridge.hpp"

extern A10TcpServer* g_tcp_server;

namespace a10_tcp
{
/// 由 ``A10PolicyTcpCliStop``（CLI ``stop``）置位；``A10PolicyTcpDriver`` 每拍检查并 ``return 0`` 退出。
std::atomic<bool> g_a10_policy_tcp_stop_requested{false};

namespace
{
constexpr int k_follower_motor_base = 0;
constexpr double k_control_hz = 500.0;
constexpr double k_dt = 1.0 / k_control_hz;
/// 这条策略的 30 步动作一共覆盖 1s，每步约 0.033s。新 chunk 到达时按观测年龄丢掉已经过去的前缀。
constexpr double k_action_chunk_horizon_s = 1.0;
constexpr int k_action_chunk_steps = 30;
constexpr double k_action_step_s = k_action_chunk_horizon_s / static_cast<double>(k_action_chunk_steps);

struct JointSegment
{
    bool active{false};
    std::vector<double> q0;
    std::vector<double> q1;
    int step_idx{0};
    int n_steps{1};
};

std::vector<double> read_follower_seven(aris::plan::Plan& plan)
{
    std::vector<double> q(7, 0.0);
    auto& motors = plan.controller()->motorPool();
    const aris::Size n_motors = motors.size();

    for (int i = 0; i < 6; ++i)
    {
        const int mi = k_follower_motor_base + i;
        if (static_cast<aris::Size>(mi) < n_motors)
        {
            q[static_cast<std::size_t>(i)] = motors[mi].actualPos();
        }
    }
    // 夹爪是 USB BusServo，不是 EtherCAT 电机 6/12。
    q[6] = g_vr_grip_actual_mm.load(std::memory_order_acquire);
    return q;
}

void apply_seven_to_motors(aris::plan::Plan& plan, const std::vector<double>& q)
{
    if (q.size() < 6)
    {
        return;
    }
    auto& motors = plan.controller()->motorPool();
    const aris::Size n_motors = motors.size();

    for (int i = 0; i < 6; ++i)
    {
        const int mi = k_follower_motor_base + i;
        if (static_cast<aris::Size>(mi) >= n_motors)
        {
            break;
        }
        motors[mi].setTargetPos(q[static_cast<std::size_t>(i)]);
    }

    // 夹爪走 USB BusServo，不占用 EtherCAT 电机。
    if (q.size() >= 7)
    {
        request_gripper_position_mm(q[6]);
    }
}

void pad_seven(std::vector<double>& v)
{
    while (v.size() < 7)
    {
        v.push_back(0.0);
    }
    if (v.size() > 7)
    {
        v.resize(7);
    }
}

int blend_steps_seven(const std::vector<double>& q0, const std::vector<double>& q1, double max_arm_vel, double max_grip_vel)
{
    return policy_blend_steps(q0, q1, max_arm_vel, max_grip_vel);
}

/// 段内插值系数：``n_steps<=1`` 时单拍直达 ``q1``；否则 ``step_idx==0`` 时 ``alpha=0`` 落在 ``q0``，
/// 避免旧式 ``(step_idx+1)/n`` 在第一拍就产生 ``(q1-q0)/n`` 的位移（常被感知为「第一步很怪」）。
double segment_lerp_alpha(int step_idx, int n_steps)
{
    if (n_steps <= 1)
    {
        return 1.0;
    }
    return static_cast<double>(step_idx) / static_cast<double>(n_steps - 1);
}

std::vector<double> lerp_q(const JointSegment& seg)
{
    const double alpha = segment_lerp_alpha(seg.step_idx, seg.n_steps);
    std::vector<double> q(7, 0.0);
    for (int i = 0; i < 7; ++i)
    {
        q[static_cast<std::size_t>(i)] = seg.q0[static_cast<std::size_t>(i)]
            + alpha * (seg.q1[static_cast<std::size_t>(i)] - seg.q0[static_cast<std::size_t>(i)]);
    }
    return q;
}

void append_policy_rtc_trace(
    const char* event,
    std::uint64_t seq,
    std::size_t remaining,
    int n_steps,
    double age_s,
    int prefix_skip,
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    const std::vector<double>& actual,
    double arm_vel,
    double grip_vel,
    double max_axis_delta,
    double max_grip_delta)
{
    if (g_tcp_server == nullptr)
        return;
    A10PolicyTrace trace;
    trace.event = event;
    trace.sequence = seq;
    trace.remaining = remaining;
    trace.n_steps = n_steps;
    trace.age_s = age_s;
    trace.prefix_skip = prefix_skip;
    trace.arm_vel = arm_vel;
    trace.grip_vel = grip_vel;
    trace.max_axis_delta = max_axis_delta;
    trace.max_grip_delta = max_grip_delta;
    std::copy_n(q0.begin(), std::min(q0.size(), trace.q0.size()), trace.q0.begin());
    std::copy_n(q1.begin(), std::min(q1.size(), trace.q1.size()), trace.q1.begin());
    std::copy_n(actual.begin(), std::min(actual.size(), trace.actual.size()), trace.actual.begin());
    g_tcp_server->record_policy_trace(trace);
}

void mout_vec7_line(aris::plan::Plan& plan, const char* label, const std::vector<double>& v)
{
    plan.mout() << label << std::fixed << std::setprecision(5);
    const int n = static_cast<int>(std::min<std::size_t>(v.size(), 7U));
    for (int i = 0; i < n; ++i)
    {
        plan.mout() << (i ? ", " : "[") << v[static_cast<std::size_t>(i)];
    }
    plan.mout() << "]" << std::defaultfloat << std::endl;
}

void start_segment(
    JointSegment& seg,
    const std::vector<double>& goal,
    aris::plan::Plan& plan,
    double max_arm_vel,
    double max_grip_vel,
    const std::vector<double>* q0_override = nullptr,
    int min_steps = 1)
{
    if (q0_override != nullptr)
    {
        seg.q0 = *q0_override;
    }
    else
    {
        seg.q0 = read_follower_seven(plan);
    }
    seg.q1 = goal;
    pad_seven(seg.q0);
    pad_seven(seg.q1);
    seg.n_steps = std::max(min_steps, blend_steps_seven(seg.q0, seg.q1, max_arm_vel, max_grip_vel));
    seg.step_idx = 0;
    seg.active = true;
}

void tick_segment_and_apply(JointSegment& seg, aris::plan::Plan& plan, std::vector<double>* reference = nullptr)
{
    if (reference != nullptr)
        *reference = lerp_q(seg);
    else
        apply_seven_to_motors(plan, lerp_q(seg));
    seg.step_idx++;
    if (seg.step_idx >= seg.n_steps)
    {
        seg.active = false;
    }
}

bool batch_keyframe_exceeds_threshold(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    double arm_max_delta_rad,
    double grip_max_delta,
    int* worst_axis,
    double* worst_abs)
{
    *worst_axis = -1;
    *worst_abs = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        const double d = std::abs(q1[static_cast<std::size_t>(i)] - q0[static_cast<std::size_t>(i)]);
        if (d > arm_max_delta_rad)
        {
            *worst_axis = i;
            *worst_abs = d;
            return true;
        }
    }
    if (q0.size() >= 7U && q1.size() >= 7U)
    {
        const double d = std::abs(q1[6] - q0[6]);
        if (d > grip_max_delta)
        {
            *worst_axis = 6;
            *worst_abs = d;
            return true;
        }
    }
    return false;
}

bool legacy_target_changed(const std::vector<double>& tq, const std::vector<double>& last)
{
    constexpr double eps = 1e-6;
    if (last.size() != 7)
    {
        return true;
    }
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(tq[static_cast<std::size_t>(i)] - last[static_cast<std::size_t>(i)]) > eps)
        {
            return true;
        }
    }
    return false;
}
}  // namespace

struct A10PolicyTcpDriver::Imp
{
    JointSegment batch_seg;
    JointSegment legacy_seg;
    std::vector<double> last_legacy_target;
    double max_arm_vel_rad_s{0.4};
    /// 夹爪绝对开度插值速度，mm/s。行程 0–85 mm。
    double max_gripper_vel{80.0};
    /// batch 队首与当前实际比较：任一角超过则丢弃该 keyframe（rad / 夹爪 mm）。
    double max_axis_delta_rad_{2.0};
    double max_grip_delta_{85.0};
    /// 与 ``A10TcpServer::policy_batch_commit_seq()`` 对齐，仅在新一批首次 keyframe 打一行 RT 日志。
    std::uint64_t last_logged_batch_commit_seq_{0};
    /// 当前执行中的 batch 序号。新 batch 到来时不清除上一段 q1，避免换段时拿实际姿态去比队首。
    std::uint64_t active_batch_commit_seq_{0};
    /// batch 连续段采用“上一段 q1 作为下一段 q0”来减少段间回拉。
    std::vector<double> last_batch_q1_;
    bool has_last_batch_q1_{false};
    /// 当前正在插值的那一段所属的 batch_seq。中途 SET 替换队列时与 commit_seq 不一致，则从当前指令改接新队首。
    std::uint64_t segment_commit_seq_{0};
    /// 已经按观测年龄丢掉过期前缀的 batch_seq。同一批只丢一次。
    std::uint64_t prefix_aligned_seq_{0};
    int last_prefix_skip_{0};
    double last_prefix_age_s_{0.0};
    /// 一旦收到并执行过 policy batch，就不再回退到 legacy ``target_q_``，避免“走几步后回撤”。
    bool policy_batch_mode_{false};
    std::uint64_t previous_cycle_ns_{0};
    double max_control_period_us_{0.0};
    double max_control_work_us_{0.0};
    PolicyCommandSmoother smoother_;
    std::vector<double> reference_command_;
    std::vector<double> previous_command_;
    std::array<double, 6> previous_velocity_{};
    bool smoothing_{false};
    double max_arm_acc_{0.5};
    double max_command_vel_{0.0};
    double max_command_acc_{0.0};

    void reset_motion_state()
    {
        batch_seg = JointSegment{};
        legacy_seg = JointSegment{};
        last_legacy_target.clear();
        last_logged_batch_commit_seq_ = 0;
        active_batch_commit_seq_ = 0;
        segment_commit_seq_ = 0;
        prefix_aligned_seq_ = 0;
        last_prefix_skip_ = 0;
        last_prefix_age_s_ = 0.0;
        last_batch_q1_.clear();
        has_last_batch_q1_ = false;
        policy_batch_mode_ = false;
        previous_cycle_ns_ = 0;
        max_control_period_us_ = 0.0;
        max_control_work_us_ = 0.0;
        reference_command_.clear();
        previous_command_.clear();
        previous_velocity_.fill(0.0);
        smoothing_ = false;
        max_command_vel_ = 0.0;
        max_command_acc_ = 0.0;
    }

    void run_one_cycle(A10PolicyTcpDriver& self)
    {
        if (g_tcp_server == nullptr)
        {
            return;
        }

        bool mid_retarget = false;
        if (batch_seg.active)
        {
            const std::uint64_t seq_now = g_tcp_server->policy_batch_commit_seq();
            if (seq_now != segment_commit_seq_)
            {
                // SET_JOINTS_BATCH 整队替换：旧队首已不在队列。从当前指令姿态接到新队列里
                // 还没过期的那一帧（下面按观测年龄丢掉前缀），不要在这里 pop。
                batch_seg.step_idx = std::max(0, batch_seg.step_idx - 1);
                last_batch_q1_ = smoothing_ ? smoother_.position() : lerp_q(batch_seg);
                pad_seven(last_batch_q1_);
                has_last_batch_q1_ = true;
                batch_seg.active = false;
                active_batch_commit_seq_ = seq_now;
                segment_commit_seq_ = seq_now;
                mid_retarget = true;
            }
            else
            {
                tick_segment_and_apply(batch_seg, self, &reference_command_);
                if (!batch_seg.active)
                {
                    g_tcp_server->pop_policy_batch_front(segment_commit_seq_);
                }
                return;
            }
        }

        std::vector<double> batch_goal;
        const std::uint64_t commit_seq_now = g_tcp_server->policy_batch_commit_seq();
        // 新 batch 仍衔接上一段 q1。SET_JOINTS_BATCH 是整队替换；若这里清空 last_q1，
        // 后续 chunk 会拿当前实际姿态比队首，跟踪滞后时容易把整段 skip 掉。
        if (commit_seq_now != active_batch_commit_seq_)
        {
            active_batch_commit_seq_ = commit_seq_now;
        }
        if (commit_seq_now != prefix_aligned_seq_)
        {
            policy_batch_mode_ = true;
            double age_s = 0.0;
            const int applied = g_tcp_server->drop_policy_batch_prefix(commit_seq_now, k_action_step_s, &age_s);
            if (applied < 0)
            {
                return;
            }
            const auto batch_status = g_tcp_server->policy_batch_status();
            if (batch_status.sequence != commit_seq_now)
                return;
            if (batch_status.smooth && !smoothing_)
            {
                auto command = read_follower_seven(self);
                auto& motors = self.controller()->motorPool();
                for (std::size_t axis = 0; axis < 6 && axis < motors.size(); ++axis)
                    command[axis] = motors[axis].targetPos();
                smoother_.reset(command);
            }
            smoothing_ = batch_status.smooth;
            prefix_aligned_seq_ = commit_seq_now;
            last_prefix_skip_ = applied;
            last_prefix_age_s_ = age_s;
            append_policy_rtc_trace("align", commit_seq_now, g_tcp_server->policy_batch_queue_size(),
                0, age_s, applied, {}, {}, read_follower_seven(self),
                max_arm_vel_rad_s, max_gripper_vel, max_axis_delta_rad_, max_grip_delta_);
            if (applied > 0)
            {
                self.mout() << "policy: chunk 对齐 age=" << std::fixed << std::setprecision(3) << age_s
                            << "s（每步 " << k_action_step_s << "s）丢掉前 " << applied
                            << " 帧，从 actions[" << applied << "] 接" << std::defaultfloat << std::endl;
            }
        }
        constexpr int k_max_batch_skips_per_tick = 64;
        int batch_skips = 0;
        bool have_valid_batch_goal = false;
        for (int attempt = 0; attempt < k_max_batch_skips_per_tick; ++attempt)
        {
            if (!g_tcp_server->peek_policy_batch_front(batch_goal, commit_seq_now))
            {
                break;
            }
            pad_seven(batch_goal);
            std::vector<double> q0snap = read_follower_seven(self);
            pad_seven(q0snap);
            const std::vector<double>& q0check = has_last_batch_q1_ ? last_batch_q1_ : q0snap;
            int bad_ax = -1;
            double bad_d = 0.0;
            if (!batch_keyframe_exceeds_threshold(
                    q0check, batch_goal, max_axis_delta_rad_, max_grip_delta_, &bad_ax, &bad_d))
            {
                have_valid_batch_goal = true;
                break;
            }
            self.mout() << "policy: [batch] skip keyframe: axis " << bad_ax << " |dq|=" << bad_d
                         << " > max_axis_delta=" << max_axis_delta_rad_ << " or max_grip_delta=" << max_grip_delta_
                         << std::endl;
            if (has_last_batch_q1_)
            {
                mout_vec7_line(self, "  q0(blend_from_last_q1)=", q0check);
            }
            else
            {
                mout_vec7_line(self, "  q0(actual)=", q0snap);
            }
            mout_vec7_line(self, "  skipped q1=", batch_goal);
            append_policy_rtc_trace("threshold_skip", commit_seq_now, g_tcp_server->policy_batch_queue_size(),
                0, last_prefix_age_s_, last_prefix_skip_, q0check, batch_goal, q0snap,
                max_arm_vel_rad_s, max_gripper_vel, max_axis_delta_rad_, max_grip_delta_);
            if (!g_tcp_server->pop_policy_batch_front(commit_seq_now))
                return;
            ++batch_skips;
            batch_goal.clear();
        }

        if (!have_valid_batch_goal)
        {
            std::vector<double> probe;
            if (g_tcp_server->peek_policy_batch_front(probe, commit_seq_now))
            {
                pad_seven(probe);
                std::vector<double> q0snap = read_follower_seven(self);
                pad_seven(q0snap);
                const std::vector<double>& q0check = has_last_batch_q1_ ? last_batch_q1_ : q0snap;
                int ax = -1;
                double ad = 0.0;
                if (batch_keyframe_exceeds_threshold(
                        q0check, probe, max_axis_delta_rad_, max_grip_delta_, &ax, &ad))
                {
                    if (self.count() % 500 == 0)
                    {
                        self.mout() << "policy: [batch] 队首仍超阈(axis " << ax << " |dq|=" << ad
                                     << ") 且本周期已达 skip 上限 " << k_max_batch_skips_per_tick
                                     << "，下周期继续尝试" << std::endl;
                    }
                }
            }
        }

        if (have_valid_batch_goal)
        {
            policy_batch_mode_ = true;
            pad_seven(batch_goal);
            const std::uint64_t commit_seq = commit_seq_now;
            if (g_tcp_server->policy_batch_commit_seq() != commit_seq)
                return;
            const std::size_t rem = g_tcp_server->policy_batch_queue_size();
            const std::vector<double>* q0_seed = has_last_batch_q1_ ? &last_batch_q1_ : nullptr;
            const double handoff_s = commit_seq != last_logged_batch_commit_seq_ ? last_prefix_age_s_ : 0.0;
            start_segment(batch_seg, batch_goal, self, max_arm_vel_rad_s, max_gripper_vel,
                q0_seed, policy_min_segment_steps(smoothing_, handoff_s));
            last_batch_q1_ = batch_seg.q1;
            has_last_batch_q1_ = true;
            segment_commit_seq_ = commit_seq;
            if (commit_seq != last_logged_batch_commit_seq_)
            {
                last_logged_batch_commit_seq_ = commit_seq;
                const std::vector<double> actual = read_follower_seven(self);
                append_policy_rtc_trace(
                    mid_retarget ? "retarget" : "segment",
                    commit_seq,
                    rem,
                    batch_seg.n_steps,
                    last_prefix_age_s_,
                    last_prefix_skip_,
                    batch_seg.q0,
                    batch_seg.q1,
                    actual,
                    max_arm_vel_rad_s, max_gripper_vel, max_axis_delta_rad_, max_grip_delta_);
                (void)batch_skips;
            }
            tick_segment_and_apply(batch_seg, self, &reference_command_);
            if (!batch_seg.active)
            {
                g_tcp_server->pop_policy_batch_front(segment_commit_seq_);
            }
            return;
        }

        // policy batch 模式下，队列暂时为空时保持当前控制，不回退到旧的 target_q_。
        if (policy_batch_mode_)
        {
            return;
        }

        std::vector<double> tq = g_tcp_server->get_target_q();
        if (tq.size() < 6)
        {
            legacy_seg.active = false;
            last_legacy_target.clear();
            return;
        }
        pad_seven(tq);

        if (legacy_seg.active)
        {
            tick_segment_and_apply(legacy_seg, self);
            if (!legacy_seg.active)
            {
                last_legacy_target = legacy_seg.q1;
            }
            return;
        }

        if (legacy_target_changed(tq, last_legacy_target))
        {
            start_segment(legacy_seg, tq, self, max_arm_vel_rad_s, max_gripper_vel);
            tick_segment_and_apply(legacy_seg, self);
            if (!legacy_seg.active)
            {
                last_legacy_target = legacy_seg.q1;
            }
            return;
        }

        apply_seven_to_motors(self, tq);
    }
};

auto A10PolicyTcpDriver::prepareNrt() -> void
{
    const double arm_acc = cmdParams().find("arm_acc") == cmdParams().end() ? 0.5 : doubleParam("arm_acc");
    if (!std::isfinite(arm_acc) || arm_acc <= 0.0)
        throw std::invalid_argument("arm_acc must be finite and positive");
    imp_->reset_motion_state();
    imp_->max_arm_acc_ = arm_acc;
    g_a10_policy_tcp_stop_requested.store(false, std::memory_order_release);
    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }
}

auto A10PolicyTcpDriver::executeRT() -> int
{
    if (g_a10_policy_tcp_stop_requested.exchange(false, std::memory_order_acq_rel))
    {
        imp_->reset_motion_state();
        if (g_tcp_server != nullptr)
        {
            g_tcp_server->clear_policy_tcp_targets_nrt();
            g_tcp_server->policy_running.store(false);
            g_tcp_server->policy_motion_settling.store(false);
            A10PolicyTrace trace;
            trace.event = "stop";
            g_tcp_server->record_policy_trace(trace);
        }
        mout() << "policy: 收到 stop 请求，退出 policy 驱动" << std::endl;
        return 0;
    }

    if (count() == 1)
    {
        if (doubleParam("arm_vel") > 1e-9)
        {
            imp_->max_arm_vel_rad_s = doubleParam("arm_vel");
        }
        if (doubleParam("grip_vel") > 1e-9)
        {
            imp_->max_gripper_vel = doubleParam("grip_vel");
        }
        if (doubleParam("max_axis_delta") > 1e-9)
        {
            imp_->max_axis_delta_rad_ = doubleParam("max_axis_delta");
        }
        if (doubleParam("max_grip_delta") > 1e-9)
        {
            imp_->max_grip_delta_ = doubleParam("max_grip_delta");
        }
        if (g_tcp_server != nullptr)
        {
            g_tcp_server->policy_running.store(true);
            append_policy_rtc_trace("start", g_tcp_server->policy_batch_commit_seq(),
                g_tcp_server->policy_batch_queue_size(), 0, 0.0, 0, {}, {}, read_follower_seven(*this),
                imp_->max_arm_vel_rad_s, imp_->max_gripper_vel, imp_->max_axis_delta_rad_, imp_->max_grip_delta_);
        }
    }

    if (g_tcp_server == nullptr)
    {
        if (count() % 2000 == 0)
        {
            mout() << "policy: g_tcp_server is null, exit plan" << std::endl;
        }
        return 0;
    }

    const auto cycle_start = anchor_monotonic_time_ns();
    if (imp_->previous_cycle_ns_ != 0)
        imp_->max_control_period_us_ = std::max(imp_->max_control_period_us_, (cycle_start - imp_->previous_cycle_ns_) / 1000.0);
    imp_->previous_cycle_ns_ = cycle_start;
    imp_->run_one_cycle(*this);
    if (!imp_->reference_command_.empty())
    {
        const auto& command = imp_->smoothing_
            ? imp_->smoother_.step(imp_->reference_command_, imp_->max_arm_vel_rad_s, imp_->max_arm_acc_)
            : imp_->reference_command_;
        apply_seven_to_motors(*this, command);
        g_tcp_server->policy_motion_settling.store(imp_->smoothing_ && !imp_->smoother_.settled(imp_->reference_command_));
        if (!imp_->previous_command_.empty())
        {
            for (std::size_t axis = 0; axis < 6; ++axis)
            {
                const double velocity = (command[axis] - imp_->previous_command_[axis]) / k_dt;
                imp_->max_command_vel_ = std::max(imp_->max_command_vel_, std::abs(velocity));
                imp_->max_command_acc_ = std::max(imp_->max_command_acc_,
                    std::abs(velocity - imp_->previous_velocity_[axis]) / k_dt);
                imp_->previous_velocity_[axis] = velocity;
            }
        }
        imp_->previous_command_ = command;
    }
    if (count() % 25 == 0)
    {
        const auto sequence = g_tcp_server->policy_batch_commit_seq();
        std::vector<double> command(7, 0.0);
        auto& motors = controller()->motorPool();
        for (std::size_t axis = 0; axis < 6 && axis < motors.size(); ++axis)
            command[axis] = motors[axis].targetPos();
        command[6] = g_vr_grip_target_mm.load(std::memory_order_acquire);
        g_tcp_server->update_policy_remaining_time(sequence, command, imp_->max_arm_vel_rad_s, imp_->max_gripper_vel);
    }
    imp_->max_control_work_us_ = std::max(imp_->max_control_work_us_, (anchor_monotonic_time_ns() - cycle_start) / 1000.0);
    if (count() % 50 == 0)
    {
        A10PolicyTrace trace;
        trace.event = "sample";
        trace.sequence = g_tcp_server->policy_batch_commit_seq();
        trace.remaining = g_tcp_server->policy_batch_queue_size();
        trace.arm_vel = imp_->max_arm_vel_rad_s;
        trace.grip_vel = imp_->max_gripper_vel;
        trace.max_axis_delta = imp_->max_axis_delta_rad_;
        trace.max_grip_delta = imp_->max_grip_delta_;
        trace.max_control_period_us = imp_->max_control_period_us_;
        trace.max_control_work_us = imp_->max_control_work_us_;
        trace.smoothing = imp_->smoothing_;
        trace.arm_acc = imp_->max_arm_acc_;
        trace.reference_lag_rad = imp_->smoothing_ && !imp_->reference_command_.empty()
            ? imp_->smoother_.lag(imp_->reference_command_) : 0.0;
        trace.max_command_vel_rad_s = imp_->max_command_vel_;
        trace.max_command_acc_rad_s2 = imp_->max_command_acc_;
        const auto actual = read_follower_seven(*this);
        std::copy_n(actual.begin(), 7, trace.actual.begin());
        auto& motors = controller()->motorPool();
        for (std::size_t axis = 0; axis < 6 && axis < motors.size(); ++axis)
            trace.q0[axis] = motors[axis].targetPos();
        trace.q0[6] = g_vr_grip_target_mm.load(std::memory_order_acquire);
        g_tcp_server->record_policy_trace(trace);
        imp_->max_control_period_us_ = 0.0;
        imp_->max_control_work_us_ = 0.0;
        imp_->max_command_vel_ = 0.0;
        imp_->max_command_acc_ = 0.0;
    }
    return count();
}

A10PolicyTcpDriver::A10PolicyTcpDriver(const std::string& name) : imp_(new Imp)
{
    aris::core::fromXmlString(
        command(),
        "<Command name=\"policy\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"arm_vel\" abbreviation=\"a\" default=\"0.05\"/>"
        "    <Param name=\"arm_acc\" default=\"0.5\"/>"
        "    <Param name=\"grip_vel\" abbreviation=\"g\" default=\"80\"/>"
        "    <Param name=\"max_axis_delta\" abbreviation=\"x\" default=\"0.5\"/>"
        "    <Param name=\"max_grip_delta\" abbreviation=\"h\" default=\"85\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10PolicyTcpDriver::~A10PolicyTcpDriver() = default;
auto A10PolicyTcpDriver::collectNrt() -> void
{
    if (g_tcp_server != nullptr)
    {
        g_tcp_server->policy_running.store(false);
        g_tcp_server->policy_motion_settling.store(false);
    }
}
KAANH_DEFINE_BIG_FOUR_CPP(A10PolicyTcpDriver)

auto A10PolicyTcpCliStop::prepareNrt() -> void
{
    for (auto& m : motorOptions())
    {
        m = aris::plan::Plan::CHECK_NONE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    }

    // vr/policy 持续占用 RT 时 executeRT 不会立刻执行；在 prepareNrt 置位，运行中的驱动下一拍即可退出。
    g_a10_policy_tcp_stop_requested.store(true, std::memory_order_release);
    request_vr_teleop_stop();
    if (g_tcp_server != nullptr)
    {
        g_tcp_server->clear_policy_tcp_targets_nrt();
    }
    mout() << "stop: 已请求结束 policy/vr 驱动并清空 TCP 缓冲" << std::endl;
    option() |= NOT_RUN_EXECUTE_FUNCTION;
}

auto A10PolicyTcpCliStop::executeRT() -> int
{
    return 0;
}

A10PolicyTcpCliStop::A10PolicyTcpCliStop(const std::string& name)
{
    (void)name;
    aris::core::fromXmlString(command(), "<Command name=\"stop\"/>");
}

A10PolicyTcpCliStop::~A10PolicyTcpCliStop() = default;
KAANH_DEFINE_BIG_FOUR_CPP(A10PolicyTcpCliStop)

}  // namespace a10_tcp

ARIS_REGISTRATION
{
    aris::core::class_<a10_tcp::A10PolicyTcpDriver>("A10PolicyTcpDriver").inherit<aris::plan::Plan>();
    aris::core::class_<a10_tcp::A10PolicyTcpCliStop>("A10PolicyTcpCliStop").inherit<aris::plan::Plan>();
}
