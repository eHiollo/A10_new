#include "a10_tcp_server.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "a10_policy_tcp_plan.hpp"
#include "a10_vr_plan.hpp"
#include "a10_vr_reset_command.hpp"
#include "kaanh/general/json.hpp"

A10TcpServer::A10TcpServer()
  : server_sockfd_(-1), running_(false)
{}

A10TcpServer::~A10TcpServer()
{
    stop();
}

bool A10TcpServer::start(uint16_t port)
{
    stop();

    server_sockfd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_sockfd_ < 0)
    {
        perror("socket");
        return false;
    }

    int opt = 1;
    if (setsockopt(server_sockfd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        return false;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (::bind(server_sockfd_, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind");
        return false;
    }

    if (::listen(server_sockfd_, 5) < 0)
    {
        perror("listen");
        return false;
    }

    running_.store(true);
    accept_thread_ = std::thread(&A10TcpServer::acceptLoop, this);
    return true;
}

void A10TcpServer::stop()
{
    running_.store(false);

    // Close server socket to unblock accept
    if (server_sockfd_ >= 0)
    {
        ::shutdown(server_sockfd_, SHUT_RDWR);
        ::close(server_sockfd_);
        server_sockfd_ = -1;
    }

    // Close all client sockets
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int sock : client_sockfds_)
        {
            ::shutdown(sock, SHUT_RDWR);
            ::close(sock);
        }
        client_sockfds_.clear();
    }

    if (accept_thread_.joinable())
        accept_thread_.join();
}

void A10TcpServer::acceptLoop()
{
    while (running_.load())
    {
        struct sockaddr_in address;
        int addrlen = sizeof(address);
        int new_socket = ::accept(server_sockfd_, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        
        if (new_socket < 0)
        {
            if (running_.load()) {
                perror("accept");
            }
            break;
        }

        std::cout << "New connection accepted: " << new_socket << std::endl;

        // Add to list
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_sockfds_.push_back(new_socket);
        }

        // Start reader thread for this client
        std::thread(&A10TcpServer::readerLoop, this, new_socket).detach();
    }
}


// 持续监听，检测header
void A10TcpServer::readerLoop(int client_sock)
{
    std::string buffer;
    buffer.reserve(1024);

    while (running_.load())
    {
        char tmp[1024];
        ssize_t n = ::recv(client_sock, tmp, sizeof(tmp), 0);
        if (n > 0)
        {
            // Completion time of the recv containing the line's final newline.
            // All lines in one recv share this timestamp, exposing batch delivery.
            const auto receive_time_ns = a10_tcp::anchor_monotonic_time_ns();
            buffer.append(tmp, tmp + n);
            size_t pos;
            // 当接收的数据出现“/n”时，表示一行数据接收完毕，转入process_line处理
            while ((pos = buffer.find('\n')) != std::string::npos)
            {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                process_line(client_sock, line, receive_time_ns);
            }
        }
        else
        {
            if (n == 0) std::cout << "Client disconnected: " << client_sock << std::endl;
            else perror("recv");
            break;
        }
    }

    // Cleanup
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = std::find(client_sockfds_.begin(), client_sockfds_.end(), client_sock);
        if (it != client_sockfds_.end())
        {
            client_sockfds_.erase(it);
        }
        ::close(client_sock);
    }
}

bool A10TcpServer::send_set_joints(const std::vector<double> &q)
{
    // 将获取的q存在robot_q_中，Aris是一毫秒推进来一次
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        robot_q_ = q;
        robot_q_updated_at_ = std::chrono::steady_clock::now();
    }

    // 不需要主动广播
    // std::string payload = "{\"q\": [";
    // for (size_t i = 0; i < q.size(); ++i)
    // {
    //     if (i) payload += ", ";
    //     payload += std::to_string(q[i]);
    // }
    // payload += "]}\n";
    // std::string cmd = std::string("SET_JOINTS ") + payload;
    // return send_line(cmd);
    return true;
}

bool A10TcpServer::send_line(const std::string &line)
{
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (client_sockfds_.empty())
        return false;

    bool any_success = false;
    const char *buf = line.c_str();
    size_t len = line.size();

    for (int sock : client_sockfds_)
    {
        ssize_t total = 0;
        size_t left = len;
        bool success = true;
        while (left > 0)
        {
            ssize_t n = ::send(sock, buf + total, left, MSG_NOSIGNAL);
            if (n <= 0)
            {
                success = false;
                break;
            }
            total += n;
            left -= n;
        }
        if (success) any_success = true;
    }
    return any_success;
}

std::vector<double> A10TcpServer::get_target_q()
{
    std::lock_guard<std::mutex> lk(q_mutex_);
    return target_q_;
    //std::cout << "get_target_q函数输出:[";
    // for ( int i = 0; i < target_q_.size(); ++i)
    // {   
    //     std::cout << target_q_[i] << (i < target_q_.size() - 1 ? ",":"" );
    // }
    // std::cout << "\n";
}

bool A10TcpServer::peek_policy_batch_front(std::vector<double>& out, std::uint64_t expected_seq)
{
    return policy_batch_.peek(out, expected_seq);
}

bool A10TcpServer::pop_policy_batch_front(std::uint64_t expected_seq)
{
    return policy_batch_.pop(expected_seq);
}

bool A10TcpServer::policy_batch_queue_empty()
{
    return policy_batch_.status().remaining == 0;
}

std::size_t A10TcpServer::policy_batch_queue_size()
{
    return policy_batch_.status().remaining;
}

std::uint64_t A10TcpServer::policy_batch_commit_seq()
{
    return policy_batch_.status().sequence;
}

void A10TcpServer::update_policy_remaining_time(std::uint64_t sequence, const std::vector<double>& command,
                                               double arm_vel, double grip_vel)
{
    policy_batch_.update_remaining_time(sequence, command, arm_vel, grip_vel);
}

int A10TcpServer::drop_policy_batch_prefix(std::uint64_t expected_seq, double action_step_s, double* age_s_out)
{
    return policy_batch_.align(expected_seq, action_step_s, age_s_out);
}

void A10TcpServer::clear_policy_tcp_targets_nrt()
{
    policy_batch_.clear();
    {
        std::lock_guard<std::mutex> lk(q_mutex_);
        target_q_.clear();
    }
    clear_ee_delta_target_nrt();
    clear_ee_anchor_target_nrt();
}

void A10TcpServer::clear_ee_delta_target_nrt()
{
    {
        std::lock_guard<std::mutex> lk(ee_delta_mutex_);
        target_ee_delta_.clear();
    }
    ee_delta_seq_.store(0, std::memory_order_release);
}

std::uint64_t A10TcpServer::ee_delta_seq() const
{
    return ee_delta_seq_.load(std::memory_order_acquire);
}

bool A10TcpServer::fetch_ee_delta_if_updated(
    std::vector<double>& out, std::uint64_t& out_seq, std::uint64_t consumed_seq)
{
    const std::uint64_t seq = ee_delta_seq_.load(std::memory_order_acquire);
    out_seq = seq;
    if (seq == 0 || seq == consumed_seq)
    {
        return false;
    }
    std::lock_guard<std::mutex> lk(ee_delta_mutex_);
    if (target_ee_delta_.size() < 7)
    {
        return false;
    }
    out = target_ee_delta_;
    return true;
}

void A10TcpServer::clear_ee_anchor_target_nrt()
{
    std::lock_guard<std::mutex> lk(ee_anchor_mutex_);
    has_ee_anchor_ = false;
    target_ee_anchor_ = a10_tcp::EeAnchorCommand{};
}

std::uint64_t A10TcpServer::ee_anchor_seq() const
{
    return ee_anchor_seq_.load(std::memory_order_acquire);
}

bool A10TcpServer::fetch_ee_anchor_if_updated(
    a10_tcp::EeAnchorCommand& out,
    std::uint64_t& out_seq,
    std::uint64_t consumed_seq)
{
    std::lock_guard<std::mutex> lk(ee_anchor_mutex_);
    const std::uint64_t seq = ee_anchor_seq_.load(std::memory_order_acquire);
    out_seq = seq;
    if (!has_ee_anchor_ || seq == 0 || seq == consumed_seq)
    {
        return false;
    }
    out = target_ee_anchor_;
    return true;
}

void A10TcpServer::send_policy_status(int client_sock)
{
    const auto log_state = policy_log_state_.snapshot();
    const auto status = policy_batch_.status();
    nlohmann::json result = {
        {"policy_protocol_version", 2}, {"controller_build", __DATE__ " " __TIME__},
        {"idle", status.remaining == 0 && !policy_motion_settling.load()}, {"remaining", status.remaining},
        {"motion_settling", policy_motion_settling.load()}, {"supports_async_smoothing", true},
        {"batch_smoothing", status.smooth},
        {"minimum_action_segment_s", (a10_tcp::policy_min_segment_steps(status.smooth) - (status.planned_trajectory ? 1 : 0)) * 0.002},
        {"batch_planned_trajectory", status.planned_trajectory},
        {"batch_jerk_limited", status.jerk_limited}, {"batch_plan_rejected", status.plan_rejected},
        {"latency_aware_handoff", true},
        {"async_handoff_planner", status.jerk_limited ? "quintic_discrete_pva_v1" : "cubic_pv_v1"},
        {"async_handoff_planners_available", {"cubic_pv_v1", "quintic_discrete_pva_v1"}},
        {"policy_vr_joint_guard", true}, {"policy_cartesian_limits_applied", false},
        {"policy_joint_speed_caps_rad_s", {a10_tcp::VrJointGuard::speed_limit(0), a10_tcp::VrJointGuard::speed_limit(1),
            a10_tcp::VrJointGuard::speed_limit(2), a10_tcp::VrJointGuard::speed_limit(3),
            a10_tcp::VrJointGuard::speed_limit(4), a10_tcp::VrJointGuard::speed_limit(5)}},
        {"policy_joint_acceleration_cap_rad_s2", a10_tcp::VrJointGuard::acceleration},
        {"remaining_time_s", status.remaining_time_s},
        {"batch_seq", status.sequence}, {"received", status.received},
        {"prefix_skip", status.prefix_skip}, {"alignment_age_s", status.alignment_age_s},
        {"handoff_selected", status.handoff_selected}, {"handoff_skip", status.handoff_skip},
        {"handoff_duration_s", status.handoff_duration_s}, {"handoff_target_time_s", status.handoff_target_time_s},
        {"handoff_late_s", status.handoff_late_s},
        {"handoff_fallback", status.handoff_fallback},
        {"aligned", status.aligned}, {"expired", status.expired},
        {"policy_running", policy_running.load()},
        {"log_enabled", log_state.enabled},
        {"log_session", std::to_string(log_state.session)}, {"action_step_s", 1.0 / 30.0},
        {"motor_checks_disabled", true},
        {"trace_dropped", policy_trace_dropped_.load()}};
    {
        std::lock_guard<std::mutex> lock(robot_q_mutex_);
        result["robot_q"] = robot_q_;
        if (robot_q_updated_at_ != std::chrono::steady_clock::time_point{})
            result["state_age_s"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - robot_q_updated_at_).count();
    }
    std::array<A10PolicyTrace, 256> traces;
    std::size_t trace_count;
    {
        std::lock_guard<std::mutex> lock(policy_trace_mutex_);
        trace_count = policy_trace_count_;
        std::copy_n(policy_traces_.begin(), trace_count, traces.begin());
        policy_trace_count_ = 0;
    }
    result["events"] = nlohmann::json::array();
    for (std::size_t index = 0; index < trace_count; ++index)
    {
        const auto& trace = traces[index];
        result["events"].push_back({
            {"event", trace.event}, {"batch_seq", trace.sequence}, {"monotonic_ns", trace.monotonic_ns},
            {"remaining", trace.remaining}, {"n_steps", trace.n_steps},
            {"age_s", trace.age_s}, {"prefix_skip", trace.prefix_skip},
            {"handoff_skip", trace.handoff_skip}, {"handoff_duration_s", trace.handoff_duration_s},
            {"handoff_target_time_s", trace.handoff_target_time_s}, {"handoff_late_s", trace.handoff_late_s},
            {"handoff_fallback", trace.handoff_fallback}, {"planned_trajectory", trace.planned_trajectory},
            {"jerk_limited", trace.jerk_limited}, {"arm_jerk", trace.arm_jerk},
            {"active_plan_remaining_s", trace.active_plan_remaining_s}, {"target_error_rad", trace.target_error_rad},
            {"arm_vel", trace.arm_vel}, {"grip_vel", trace.grip_vel},
            {"max_axis_delta", trace.max_axis_delta}, {"max_grip_delta", trace.max_grip_delta},
            {"max_control_period_us", trace.max_control_period_us}, {"max_control_work_us", trace.max_control_work_us},
            {"smoothing", trace.smoothing}, {"arm_acc", trace.arm_acc},
            {"reference_lag_rad", trace.reference_lag_rad},
            {"max_command_vel_rad_s", trace.max_command_vel_rad_s},
            {"max_command_acc_rad_s2", trace.max_command_acc_rad_s2},
            {"max_command_jerk_rad_s3", trace.max_command_jerk_rad_s3},
            {"command_velocity_rad_s", trace.command_velocity_rad_s},
            {"command_acceleration_rad_s2", trace.command_acceleration_rad_s2},
            {"max_following_error_rad", trace.max_following_error_rad},
            {"q0", trace.q0}, {"q1", trace.q1}, {"actual", trace.actual}});
    }
    const std::string payload = result.dump() + "\n";

    ssize_t total = 0;
    const char* buf = payload.c_str();
    size_t left = payload.size();
    while (left > 0)
    {
        ssize_t n = ::send(client_sock, buf + total, left, MSG_NOSIGNAL);
        if (n <= 0)
            break;
        total += n;
        left -= n;
    }
}

void A10TcpServer::record_policy_trace(A10PolicyTrace trace)
{
    trace.monotonic_ns = a10_tcp::anchor_monotonic_time_ns();
    std::unique_lock<std::mutex> lock(policy_trace_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || policy_trace_count_ == policy_traces_.size())
    {
        ++policy_trace_dropped_;
        return;
    }
    policy_traces_[policy_trace_count_++] = trace;
}

static void send_line_to_client(int client_sock, const std::string& line)
{
    ssize_t total = 0;
    const char* buf = line.c_str();
    size_t left = line.size();
    while (left > 0)
    {
        ssize_t n = ::send(client_sock, buf + total, left, MSG_NOSIGNAL);
        if (n <= 0)
            break;
        total += n;
        left -= n;
    }
}

void A10TcpServer::send_leader_state(int client_sock)
{
    std::vector<double> current_q;
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        current_q = robot_q_;
    }
    
    if (current_q.size() < 13)
    {
        current_q.resize(13, 0.0);
    }

    // 双臂时 6..11 是主臂；单臂没有第二臂，发 0。
    std::string payload = "{\"q\": [";
    for (size_t i = 0; i < 6; ++i)
    {
        if (i) payload += ", ";
        payload += std::to_string(current_q[i + 6]);
    }
    payload += "]}\n";
    
    // Send directly to the requesting client
    ssize_t total = 0;
    const char *buf = payload.c_str();
    size_t left = payload.size();
    while (left > 0)
    {
        ssize_t n = ::send(client_sock, buf + total, left, MSG_NOSIGNAL);
        if (n <= 0) break;
        total += n;
        left -= n;
    }
}

void A10TcpServer::send_follower_state(int client_sock)
{
    std::vector<double> current_q;
    double state_age_s = -1.0;
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        current_q = robot_q_;
        if (robot_q_updated_at_ != std::chrono::steady_clock::time_point{})
            state_age_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - robot_q_updated_at_).count();
    }
    
    if (current_q.size() < 13)
    {
        current_q.resize(13, 0.0);
    }

    //发送七维数据出去
    size_t start_idx = 0;
    size_t end_idx = 7;

    std::string payload = "{\"q\": [";
    for (size_t i = start_idx; i < end_idx-1; ++i)
    {
        if (i > start_idx) payload += ", ";
        payload += std::to_string(current_q[i]);
    }

    //将第12维度的舵机数据传出去
    payload += ", ";
    payload += std::to_string(current_q[12]);
    payload += "],\"state_valid\":" + std::string(state_age_s >= 0.0 ? "true" : "false")
        + ",\"state_age_s\":" + std::to_string(state_age_s) + "}\n";
    
    // Send directly to the requesting client
    ssize_t total = 0;
    const char *buf = payload.c_str();
    size_t left = payload.size();
    while (left > 0)
    {
        ssize_t n = ::send(client_sock, buf + total, left, MSG_NOSIGNAL);
        if (n <= 0) break;
        total += n;
        left -= n;
    }
}

void A10TcpServer::process_line(
    int client_sock, const std::string &line, std::uint64_t receive_time_ns)
{
    // 响应“”GET_LEADER_STATE”请求
    if (line.find("GET_LEADER_STATE") != std::string::npos)
    {
        send_leader_state(client_sock);
        return;
    }

    // 响应“”GET_FOLLOWER_STATE”请求
    if (line.find("GET_FOLLOWER_STATE") != std::string::npos)
    {
        send_follower_state(client_sock);
        return;
    }

    if (line.find("GET_POLICY_STATUS") != std::string::npos)
    {
        send_policy_status(client_sock);
        return;
    }

    // 异步桥接优雅停止：清空 batch 队列，并让正在跑的 policy 下一拍退出，停在当前位置。
    if (line.find("STOP_POLICY") != std::string::npos)
    {
        a10_tcp::g_a10_policy_tcp_stop_requested.store(true, std::memory_order_release);
        policy_log_state_.stop();
        clear_policy_tcp_targets_nrt();
        send_line_to_client(client_sock, "{\"stopped\":true}\n");
        return;
    }

    // Same flag as the Shell ``reset`` command: vr_vel homes without leaving RT.
    // The VR client sends this line; a prefix check on "RESET" dropped "reset".
    if (a10_tcp::is_vr_reset_command(line))
    {
        a10_tcp::request_vr_init();
        std::cout << "reset: requested, vr_vel RT will home" << std::endl;
        return;
    }

    // A2.2: anchored pose mailbox. vr_vel consumes it as diagnostics only.
    if (line.find("SET_EE_ANCHOR") != std::string::npos)
    {
        a10_tcp::EeAnchorCommand command;
        std::string error;
        if (!a10_tcp::parse_ee_anchor_line(line, command, &error))
        {
            std::cout << "SET_EE_ANCHOR parse error: " << error << std::endl;
            return;
        }
        {
            std::lock_guard<std::mutex> lk(ee_anchor_mutex_);
            command.robot_receive_time_ns = receive_time_ns;
            command.robot_publish_time_ns = a10_tcp::anchor_monotonic_time_ns();
            target_ee_anchor_ = std::move(command);
            has_ee_anchor_ = true;
            (void)ee_anchor_seq_.fetch_add(1, std::memory_order_acq_rel);
        }
        return;
    }

    // LeRobot 遥操作：tool/ee 系末端增量，actions 为一维 7 向量（m + rotvec rad + grip）。
    if (line.find("SET_EE_DELTA") != std::string::npos)
    {
        const size_t j0 = line.find('{');
        if (j0 == std::string::npos)
        {
            std::cout << "SET_EE_DELTA: missing JSON object" << std::endl;
            return;
        }
        try
        {
            const nlohmann::json j = nlohmann::json::parse(line.substr(j0));
            if (!j.contains("actions") || !j["actions"].is_array())
            {
                std::cout << "SET_EE_DELTA: need \"actions\" array" << std::endl;
                return;
            }
            std::vector<double> v;
            for (const auto& x : j["actions"])
            {
                if (x.is_number())
                {
                    v.push_back(x.get<double>());
                }
            }
            while (v.size() < 7)
            {
                v.push_back(0.0);
            }
            if (v.size() > 7)
            {
                v.resize(7);
            }
            {
                std::lock_guard<std::mutex> lk(ee_delta_mutex_);
                target_ee_delta_ = std::move(v);
            }
            (void)ee_delta_seq_.fetch_add(1, std::memory_order_acq_rel);
        }
        catch (const std::exception& e)
        {
            std::cout << "SET_EE_DELTA parse error: " << e.what() << std::endl;
        }
        return;
    }

    // 整段模型 actions：一行 JSON，写入 ``target_q_batch_``，由 RT 逐步执行；不覆盖 ``target_q_``。
    if (line.find("SET_JOINTS_BATCH") != std::string::npos)
    {
        const size_t j0 = line.find('{');
        if (j0 == std::string::npos)
        {
            std::cout << "SET_JOINTS_BATCH: missing JSON object" << std::endl;
            send_line_to_client(client_sock, "{\"accepted\":false,\"queued\":0}\n");
            return;
        }
        try
        {
            const nlohmann::json j = nlohmann::json::parse(line.substr(j0));
            if (!j.contains("actions") || !j["actions"].is_array())
            {
                std::cout << "SET_JOINTS_BATCH: need \"actions\" array" << std::endl;
                send_line_to_client(client_sock, "{\"accepted\":false,\"queued\":0}\n");
                return;
            }
            std::deque<std::vector<double>> new_batch;
            for (const auto& row : j["actions"])
            {
                if (!row.is_array() || row.size() != 7)
                    throw std::invalid_argument("each action must have exactly seven numbers");
                std::vector<double> v;
                for (const auto& x : row)
                {
                    if (!x.is_number())
                        throw std::invalid_argument("action contains a non-number");
                    v.push_back(x.get<double>());
                }
                new_batch.push_back(std::move(v));
            }
            const size_t n = new_batch.size();
            double obs_age_s = 0.0;
            if (j.contains("obs_age_s"))
            {
                if (!j["obs_age_s"].is_number())
                    throw std::invalid_argument("invalid obs_age_s");
                obs_age_s = j["obs_age_s"].get<double>();
            }
            std::optional<std::uint64_t> expected_sequence;
            if (j.contains("expected_batch_seq"))
            {
                if (!j["expected_batch_seq"].is_number_unsigned())
                    throw std::invalid_argument("invalid expected_batch_seq");
                expected_sequence = j["expected_batch_seq"].get<std::uint64_t>();
            }
            if (j.contains("smooth") && !j["smooth"].is_boolean())
                throw std::invalid_argument("smooth must be boolean");
            const bool smooth = j.value("smooth", false);
            const auto batch_seq = policy_batch_.replace(std::move(new_batch), obs_age_s,
                a10_tcp::PolicyBatchQueue::Clock::now(), expected_sequence, smooth);
            send_line_to_client(client_sock,
                nlohmann::json({{"accepted", true}, {"queued", n}, {"batch_seq", batch_seq}, {"smooth", smooth}}).dump() + "\n");
        }
        catch (const std::exception& e)
        {
            std::cout << "SET_JOINTS_BATCH parse error: " << e.what() << std::endl;
            send_line_to_client(client_sock, "{\"accepted\":false,\"queued\":0}\n");
        }
        return;
    }


    // 响应“SET_JOINTS {....}”请求
    size_t qpos = line.find("\"q\"");
    if (qpos == std::string::npos)
    {
        std::cout << "Received unknown command: " << line << std::endl;
        return;
    }
    size_t lbr = line.find('[', qpos);
    size_t rbr = line.find(']', lbr == std::string::npos ? 0 : lbr);
    if (lbr == std::string::npos || rbr == std::string::npos || rbr <= lbr)
        return;

    std::string arr = line.substr(lbr + 1, rbr - lbr - 1);
    std::vector<double> parsed;
    size_t idx = 0;
    //逗号分隔符解析器，用于解析收到的JSON数据
    while (idx < arr.size())
    {
        while (idx < arr.size() && isspace((unsigned char)arr[idx])) ++idx;
        if (idx >= arr.size()) break;
        size_t comma = arr.find(',', idx);
        std::string token;
        if (comma == std::string::npos)
        {
            token = arr.substr(idx);
            idx = arr.size();
        }
        else
        {
            token = arr.substr(idx, comma - idx);
            idx = comma + 1;
        }
        size_t b = 0, e = token.size();
        while (b < e && isspace((unsigned char)token[b])) ++b;
        while (e > b && isspace((unsigned char)token[e-1])) --e;
        if (e <= b) continue;
        std::string numstr = token.substr(b, e-b);
        try
        {
            double v = std::stod(numstr);
            parsed.push_back(v);
        }
        catch (...) { }
    }

    //将解析的数据存在了私有变量q_中，供外部获取
    if (!parsed.empty())
    {
        std::lock_guard<std::mutex> lk(q_mutex_);
        target_q_ = parsed;

        // static int printcount = 0;
        // if( printcount++ % 1000 ==0)
        // {   std::cout << "Received target_q:[";
        //     for ( int i = 0; i < target_q_.size(); ++i)
        //     {
        //         std::cout << target_q_[i] << (i < target_q_.size() - 1 ? ",":"" );
        //     }
        //     std::cout << "]\n";
        // }
    }
}
