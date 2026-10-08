#ifndef A10_TCP_SERVER_HPP_
#define A10_TCP_SERVER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "a10_anchor_protocol.hpp"
#include "a10_policy_batch.hpp"

struct A10PolicyTrace
{
    const char* event{"segment"};
    std::uint64_t sequence{0};
    std::uint64_t monotonic_ns{0};
    std::size_t remaining{0};
    int n_steps{0};
    int prefix_skip{0};
    double age_s{0.0};
    double arm_vel{0.0};
    double grip_vel{0.0};
    double max_axis_delta{0.0};
    double max_grip_delta{0.0};
    double max_control_period_us{0.0};
    double max_control_work_us{0.0};
    bool smoothing{false};
    double arm_acc{0.0};
    double reference_lag_rad{0.0};
    double max_command_vel_rad_s{0.0};
    double max_command_acc_rad_s2{0.0};
    std::array<double, 7> q0{};
    std::array<double, 7> q1{};
    std::array<double, 7> actual{};
};

/// Thread-safe TCP server for A10 robot (line-delimited JSON protocol).
class A10TcpServer
{
public:
    A10TcpServer();
    ~A10TcpServer();

    bool start(uint16_t port);
    void stop();

    bool send_set_joints(const std::vector<double>& q);
    std::vector<double> get_target_q();

    bool peek_policy_batch_front(std::vector<double>& out, std::uint64_t expected_seq);
    bool pop_policy_batch_front(std::uint64_t expected_seq);
    bool policy_batch_queue_empty();
    std::size_t policy_batch_queue_size();
    std::uint64_t policy_batch_commit_seq();
    a10_tcp::PolicyBatchStatus policy_batch_status() const { return policy_batch_.status(); }
    void update_policy_remaining_time(std::uint64_t sequence, const std::vector<double>& command,
                                      double arm_vel, double grip_vel);
    /// 同一把锁里按观测年龄加接收后的等待时间丢掉过期帧，整段过期则全部丢弃。
    /// ``expected_seq`` 与当前 batch 不一致时返回 -1，调用方下个周期再对齐。
    int drop_policy_batch_prefix(std::uint64_t expected_seq, double action_step_s, double* age_s_out);
    void clear_policy_tcp_targets_nrt();
    void record_policy_trace(A10PolicyTrace trace);
    std::atomic<bool> policy_running{false};
    std::atomic<bool> policy_motion_settling{false};

    bool fetch_ee_delta_if_updated(
        std::vector<double>& out, std::uint64_t& out_seq, std::uint64_t consumed_seq);
    void clear_ee_delta_target_nrt();
    std::uint64_t ee_delta_seq() const;

    bool fetch_ee_anchor_if_updated(
        a10_tcp::EeAnchorCommand& out,
        std::uint64_t& out_seq,
        std::uint64_t consumed_seq);
    void clear_ee_anchor_target_nrt();
    std::uint64_t ee_anchor_seq() const;

private:
    void acceptLoop();
    void readerLoop(int client_sock);
    bool send_line(const std::string& line);
    void send_policy_status(int client_sock);
    void send_leader_state(int client_sock);
    void send_follower_state(int client_sock);
    void process_line(int client_sock, const std::string& line, std::uint64_t receive_time_ns);

    int server_sockfd_;
    std::vector<int> client_sockfds_;
    std::atomic<bool> running_;
    std::thread accept_thread_;

    std::mutex clients_mutex_;

    std::mutex q_mutex_;
    std::vector<double> target_q_;

    std::mutex robot_q_mutex_;
    std::vector<double> robot_q_;
    std::chrono::steady_clock::time_point robot_q_updated_at_{};

    a10_tcp::PolicyBatchQueue policy_batch_;
    std::mutex policy_trace_mutex_;
    std::array<A10PolicyTrace, 256> policy_traces_{};
    std::size_t policy_trace_count_{0};
    std::atomic<std::uint64_t> policy_trace_dropped_{0};

    std::mutex ee_delta_mutex_;
    std::vector<double> target_ee_delta_;
    std::atomic<std::uint64_t> ee_delta_seq_{0};

    std::mutex ee_anchor_mutex_;
    a10_tcp::EeAnchorCommand target_ee_anchor_;
    bool has_ee_anchor_{false};
    std::atomic<std::uint64_t> ee_anchor_seq_{0};
};

extern A10TcpServer* g_tcp_server;

#endif  // A10_TCP_SERVER_HPP_
