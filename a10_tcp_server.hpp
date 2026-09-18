#ifndef A10_TCP_SERVER_HPP_
#define A10_TCP_SERVER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
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

    bool peek_policy_batch_front(std::vector<double>& out);
    void pop_policy_batch_front();
    bool policy_batch_queue_empty();
    std::size_t policy_batch_queue_size();
    std::uint64_t policy_batch_commit_seq();
    void clear_policy_tcp_targets_nrt();

    bool fetch_ee_delta_if_updated(
        std::vector<double>& out, std::uint64_t& out_seq, std::uint64_t consumed_seq);
    void clear_ee_delta_target_nrt();
    std::uint64_t ee_delta_seq() const;

    bool fetch_ee_anchor_if_updated(
        a10_tcp::EeAnchorCommand& out,
        std::uint64_t& out_seq,
        std::uint64_t consumed_seq);
    void clear_ee_anchor_target_nrt();

private:
    void acceptLoop();
    void readerLoop(int client_sock);
    bool send_line(const std::string& line);
    void send_policy_status(int client_sock);
    void send_leader_state(int client_sock);
    void send_follower_state(int client_sock);
    void process_line(int client_sock, const std::string& line);

    int server_sockfd_;
    std::vector<int> client_sockfds_;
    std::atomic<bool> running_;
    std::thread accept_thread_;

    std::mutex clients_mutex_;

    std::mutex q_mutex_;
    std::vector<double> target_q_;

    std::mutex robot_q_mutex_;
    std::vector<double> robot_q_;

    std::mutex batch_mutex_;
    std::deque<std::vector<double>> target_q_batch_;
    std::uint64_t batch_commit_seq_{0};

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
