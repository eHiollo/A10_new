#include "a10_tcp_server.hpp"

#include <algorithm>

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

bool A10TcpServer::peek_policy_batch_front(std::vector<double>& out)
{
    std::lock_guard<std::mutex> lk(batch_mutex_);
    if (target_q_batch_.empty())
    {
        return false;
    }
    out = target_q_batch_.front();
    return true;
}

void A10TcpServer::pop_policy_batch_front()
{
    std::lock_guard<std::mutex> lk(batch_mutex_);
    if (!target_q_batch_.empty())
    {
        target_q_batch_.pop_front();
    }
}

bool A10TcpServer::policy_batch_queue_empty()
{
    std::lock_guard<std::mutex> lk(batch_mutex_);
    return target_q_batch_.empty();
}

std::size_t A10TcpServer::policy_batch_queue_size()
{
    std::lock_guard<std::mutex> lk(batch_mutex_);
    return target_q_batch_.size();
}

std::uint64_t A10TcpServer::policy_batch_commit_seq()
{
    std::lock_guard<std::mutex> lk(batch_mutex_);
    return batch_commit_seq_;
}

void A10TcpServer::clear_policy_tcp_targets_nrt()
{
    {
        std::lock_guard<std::mutex> lk(batch_mutex_);
        target_q_batch_.clear();
        ++batch_commit_seq_;
    }
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
    size_t rem = 0;
    std::uint64_t seq = 0;
    {
        std::lock_guard<std::mutex> lk(batch_mutex_);
        rem = target_q_batch_.size();
        seq = batch_commit_seq_;
    }
    const char* idle_str = (rem == 0) ? "true" : "false";
    std::string payload = std::string("{\"idle\":") + idle_str + ",\"remaining\":" + std::to_string(rem)
        + ",\"batch_seq\":" + std::to_string(seq) + "}\n";

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
    {
        std::lock_guard<std::mutex> lk(robot_q_mutex_);
        current_q = robot_q_;
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
            const std::size_t actions_array_len = j["actions"].size();
            std::size_t row_index = 0;
            for (const auto& row : j["actions"])
            {
                if (!row.is_array())
                {
                    std::cout << "SET_JOINTS_BATCH: skip actions[" << row_index << "] (not JSON array)\n";
                    ++row_index;
                    continue;
                }
                std::vector<double> v;
                for (const auto& x : row)
                {
                    if (x.is_number())
                    {
                        v.push_back(x.get<double>());
                    }
                }
                if (v.empty())
                {
                    std::cout << "SET_JOINTS_BATCH: skip actions[" << row_index << "] (no numeric elements)\n";
                    ++row_index;
                    continue;
                }
                while (v.size() < 7)
                {
                    v.push_back(0.0);
                }
                if (v.size() > 7)
                {
                    v.resize(7);
                }
                new_batch.push_back(std::move(v));
                ++row_index;
            }
            const size_t n = new_batch.size();
            if (actions_array_len != n)
            {
                std::cout << "SET_JOINTS_BATCH: warning JSON actions length " << actions_array_len
                          << " != parsed keyframes " << n << " (skipped rows → 队首可能不是模型 actions[0])\n";
            }
            if (!new_batch.empty())
            {
                const auto& f = new_batch.front();
                std::cout << "SET_JOINTS_BATCH: queued n=" << n << " queue_front[0..6]=";
                for (int i = 0; i < 7; ++i)
                {
                    std::cout << (i ? "," : "") << f[static_cast<std::size_t>(i)];
                }
                std::cout << std::endl;
            }
            {
                std::lock_guard<std::mutex> lk(batch_mutex_);
                target_q_batch_ = std::move(new_batch);
                ++batch_commit_seq_;
            }
            send_line_to_client(client_sock,
                std::string("{\"accepted\":true,\"queued\":") + std::to_string(n) + "}\n");
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
