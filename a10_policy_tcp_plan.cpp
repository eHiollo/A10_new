#include "a10_policy_tcp_plan.hpp"

#include "kaanh/general/macro.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <vector>

#include "a10_tcp_server.hpp"
#include "a10_vr_plan.hpp"

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
    // 夹爪在 ethercat 上是 motor 12；没有该电机时保持 0。
    // 禁止回退到 motor 6：那是主臂关节 0，|q6| 经常 > max_grip_delta(0.6)，
    // 会在一个 RT 周期内把整段 follower batch 全部 skip 掉，机械臂完全不动。
    if (static_cast<aris::Size>(12) < n_motors)
    {
        q[6] = motors[12].actualPos();
    }
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

    if (q.size() >= 7 && static_cast<aris::Size>(12) < n_motors)
    {
        motors[12].setTargetPos(q[6]);
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
    int n = 1;
    for (int i = 0; i < 6; ++i)
    {
        const double dq = std::abs(q1[static_cast<std::size_t>(i)] - q0[static_cast<std::size_t>(i)]);
        const int steps = static_cast<int>(std::ceil(dq / (max_arm_vel * k_dt)));
        n = std::max(n, std::max(1, steps));
    }
    if (q0.size() >= 7 && q1.size() >= 7)
    {
        const double dq = std::abs(q1[6] - q0[6]);
        const int steps = static_cast<int>(std::ceil(dq / (max_grip_vel * k_dt)));
        n = std::max(n, std::max(1, steps));
    }
    return n;
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

void tick_segment_and_apply(JointSegment& seg, aris::plan::Plan& plan)
{
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
    (void)grip_max_delta;
    // 夹爪不参与 skip：无夹爪时 goal[6]=0，q0[6] 若读到其它电机会把整段轨迹一拍清掉。
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
    double max_gripper_vel{0.15};
    /// batch 队首与当前实际比较：任一角超过则丢弃该 keyframe（rad / 夹爪同单位）。
    double max_axis_delta_rad_{2.0};
    double max_grip_delta_{0.6};
    /// 与 ``A10TcpServer::policy_batch_commit_seq()`` 对齐，仅在新一批首次 keyframe 打一行 RT 日志。
    std::uint64_t last_logged_batch_commit_seq_{0};
    /// 当前执行中的 batch 序号；当序号变化时重置连续段衔接状态。
    std::uint64_t active_batch_commit_seq_{0};
    /// batch 连续段采用“上一段 q1 作为下一段 q0”来减少段间回拉。
    std::vector<double> last_batch_q1_;
    bool has_last_batch_q1_{false};
    /// 一旦收到并执行过 policy batch，就不再回退到 legacy ``target_q_``，避免“走几步后回撤”。
    bool policy_batch_mode_{false};

    void reset_motion_state()
    {
        batch_seg = JointSegment{};
        legacy_seg = JointSegment{};
        last_legacy_target.clear();
        last_logged_batch_commit_seq_ = 0;
        active_batch_commit_seq_ = 0;
        last_batch_q1_.clear();
        has_last_batch_q1_ = false;
        policy_batch_mode_ = false;
    }

    void run_one_cycle(A10PolicyTcpDriver& self)
    {
        if (g_tcp_server == nullptr)
        {
            return;
        }

        if (batch_seg.active)
        {
            tick_segment_and_apply(batch_seg, self);
            if (!batch_seg.active)
            {
                g_tcp_server->pop_policy_batch_front();
            }
            return;
        }

        std::vector<double> batch_goal;
        const std::uint64_t commit_seq_now = g_tcp_server->policy_batch_commit_seq();
        // 新 batch 仍衔接上一段 q1。SET_JOINTS_BATCH 是整队替换，若这里清空 last_q1，
        // 后续 chunk 会拿「当前实际姿态」比队首，前期几乎不动时队首相对复位姿态越来越大，触发 skip 雪崩。
        if (commit_seq_now != active_batch_commit_seq_)
        {
            active_batch_commit_seq_ = commit_seq_now;
        }
        constexpr int k_max_batch_skips_per_tick = 64;
        int batch_skips = 0;
        bool have_valid_batch_goal = false;
        for (int attempt = 0; attempt < k_max_batch_skips_per_tick; ++attempt)
        {
            if (!g_tcp_server->peek_policy_batch_front(batch_goal))
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
            g_tcp_server->pop_policy_batch_front();
            ++batch_skips;
            batch_goal.clear();
        }

        if (!have_valid_batch_goal)
        {
            std::vector<double> probe;
            if (g_tcp_server->peek_policy_batch_front(probe))
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
            const std::uint64_t commit_seq = g_tcp_server->policy_batch_commit_seq();
            const std::size_t rem = g_tcp_server->policy_batch_queue_size();
            const std::vector<double>* q0_seed = has_last_batch_q1_ ? &last_batch_q1_ : nullptr;
            // batch 段间衔接最少 2 个实时周期，避免单拍跳变导致的“顿挫”。
            start_segment(batch_seg, batch_goal, self, max_arm_vel_rad_s, max_gripper_vel, q0_seed, 2);
            last_batch_q1_ = batch_seg.q1;
            has_last_batch_q1_ = true;
            if (commit_seq != last_logged_batch_commit_seq_)
            {
                last_logged_batch_commit_seq_ = commit_seq;
                self.mout() << "policy: [batch] 新一批 keyframe#0 batch_seq=" << commit_seq << " remaining=" << rem
                             << " n_steps=" << batch_seg.n_steps
                             << " alpha@step0=" << segment_lerp_alpha(0, batch_seg.n_steps);
                if (batch_skips > 0)
                {
                    self.mout() << " (本周期已跳过 " << batch_skips << " 个异常 keyframe)";
                }
                self.mout() << std::endl;
                mout_vec7_line(self, "  q0(actual)=", batch_seg.q0);
                mout_vec7_line(self, "  q1(queue_front)=", batch_seg.q1);
                mout_vec7_line(self, "  first_cmd=", lerp_q(batch_seg));
            }
            tick_segment_and_apply(batch_seg, self);
            if (!batch_seg.active)
            {
                g_tcp_server->pop_policy_batch_front();
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
    imp_->reset_motion_state();
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
    }

    if (g_tcp_server == nullptr)
    {
        if (count() % 2000 == 0)
        {
            mout() << "policy: g_tcp_server is null, exit plan" << std::endl;
        }
        return 0;
    }

    imp_->run_one_cycle(*this);
    return count();
}

A10PolicyTcpDriver::A10PolicyTcpDriver(const std::string& name) : imp_(new Imp)
{
    aris::core::fromXmlString(
        command(),
        "<Command name=\"policy\">"
        "  <GroupParam name=\"group_param\">"
        "    <Param name=\"arm_vel\" abbreviation=\"a\" default=\"0.05\"/>"
        "    <Param name=\"grip_vel\" abbreviation=\"g\" default=\"0.15\"/>"
        "    <Param name=\"max_axis_delta\" abbreviation=\"x\" default=\"0.5\"/>"
        "    <Param name=\"max_grip_delta\" abbreviation=\"h\" default=\"0.6\"/>"
        "  </GroupParam>"
        "</Command>");
}

A10PolicyTcpDriver::~A10PolicyTcpDriver() = default;
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
