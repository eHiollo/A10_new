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

    /// RT 线程每周期调用，写入当前末端位姿(pe: x,y,z,rx,ry,rz, 单位 m / rad)。
    void update_ee_pose(const double pe[6]);
    /// 读取当前末端位姿(供 GET_EE_STATE 使用)；未更新过返回 false。
    bool get_ee_pose(std::vector<double>& out);

    /// VR RT 线程轮询：若 SET_EE_TARGET 有新目标则取出(7D: x,y,z,rx,ry,rz,gripper)。
    bool fetch_ee_target_if_updated(
        std::vector<double>& out, std::uint64_t& out_seq, std::uint64_t consumed_seq);
    void clear_ee_target_nrt();
    std::uint64_t ee_target_seq() const;

private:
    void acceptLoop();
    void readerLoop(int client_sock);
    bool send_line(const std::string& line);
    void send_policy_status(int client_sock);
    void send_leader_state(int client_sock);
    void send_follower_state(int client_sock);
    void send_ee_state(int client_sock);
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

    /// RT 写入、GET_EE_STATE 读取的当前末端位姿(pe: x,y,z,rx,ry,rz)。
    std::mutex ee_pose_mutex_;
    std::vector<double> current_ee_pe_;
    bool ee_pose_valid_{false};

    /// SET_EE_TARGET 写入、VR RT 读取的绝对末端目标(7D: x,y,z,rx,ry,rz,gripper)。
    std::mutex ee_target_mutex_;
    std::vector<double> target_ee_absolute_;
    std::atomic<std::uint64_t> ee_target_seq_{0};
};

extern A10TcpServer* g_tcp_server;

#endif  // A10_TCP_SERVER_HPP_
