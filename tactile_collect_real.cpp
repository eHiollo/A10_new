#include "tactile_collect_real.hpp"
#include "gravcomp.hpp"

#include <array>
#include <numeric>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <iostream>
#include <utility>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cctype>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <sys/select.h>

using namespace std;

namespace tactile_collect_real
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;
    }

    enum CollectStage
    {
        STAGE_INIT_BASELINE = 0,
        STAGE_MOVE_TO_PREAPPROACH = 1,
        STAGE_MOVE_TO_CONTACT_SAMPLE = 2,
        STAGE_SETTLE = 3,
        STAGE_COLLECT = 4,
        STAGE_RETREAT = 5,
        STAGE_FINISH = 6,
        STAGE_ROT_BASELINE_STABILIZE = 7,
        STAGE_RECOVER_TO_BASELINE = 8
    };

    struct TactileCollectReal::Imp
    {
        int m_{ 0 };

        bool file_ready{ false };
        bool sample_set_built{ false };
        bool baseline_ready{ false };

        int stage{ STAGE_INIT_BASELINE };
        int stage_wait_count{ 0 };
        int settle_counter{ 0 };
        int collect_counter{ 0 };
        int sample_index{ 0 };
        int total_record_count{ 0 };

        // Extra stabilization after a rotational sample retreats to the baseline pose.
        // This prevents one rotational sample from contaminating the next sample.
        int rot_baseline_stabilize_counter{ 0 };

        // Adaptive waiting monitor.
        // The program no longer fails merely because a fixed short time has passed.
        // It keeps waiting while the pose error is still improving, and only gives up
        // when there is no meaningful progress for a continuous period or a hard safety
        // upper bound is reached.
        int no_progress_count{ 0 };
        double best_pos_err{ 1.0e100 };
        double best_rot_err{ 1.0e100 };

        // 0: no contact, 1: light contact, 2: hold-force contact, 3: danger contact.
        int current_contact_flag{ 0 };

        int baseline_count{ 0 };
        int baseline_sample_count{ 200 };

        std::string csv_dir{ "/home/kaanh/tactile_collect_data" };
        std::string csv_path;
        std::ofstream csv;
        std::string log_path;
        std::ofstream log;

        double arm1_p_vector[9]{ 0 };
        double arm1_l_vector[9]{ 0 };
        double arm2_p_vector[9]{ 0 };
        double arm2_l_vector[9]{ 0 };

        double raw_bias_sum[6]{ 0 };
        double raw_bias[6]{ 0 };

        std::array<double, 10> force_buffer[6]{};
        int buffer_index[6]{ 0 };

        double perfect_pose[6]{ 0 };
        double init_pose[6]{ 0 };
        double current_target[6]{ 0 };
        double actual_pose[6]{ 0 };
        double transformed_force[6]{ 0 };

        double collect_pose_sum[6]{ 0 };
        int collect_pose_valid_count{ 0 };

        std::vector<std::array<double, 6>> sample_offsets;
        std::vector<std::string> sample_names;

        // TCP bridge to the Windows DM-Tac collector.
        // C++ is the TCP client. Windows Python is the TCP server.
        int tcp_enable{ 1 };
        std::string tcp_host{ "192.168.1.7" };
        int tcp_port{ 50007 };
        int tcp_connect_timeout_ms{ 3000 };
        int tcp_sample_frames{ 5 };
        int tcp_collect_timeout_count{ 6000 }; // 12 s when robot cycle is 2 ms
        int tcp_sock{ -1 };
        bool tcp_connected{ false };
        bool tcp_request_active{ false };
        int tcp_request_wait_count{ 0 };
        int tcp_active_sample_id{ -1 };
        int tcp_active_record_id{ -1 };
        int tcp_active_contact_flag{ 0 };
        std::string tcp_rx_buffer;
        std::string tcp_last_ack;
    };

    auto makeTimestampString() -> std::string
    {
        const auto now = std::chrono::system_clock::now();
        const auto tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now{};
        localtime_r(&tt, &tm_now);
        std::ostringstream oss;
        oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    auto ensureDirExists(const std::string& dir) -> bool
    {
        if (dir.empty()) return false;
        if (::mkdir(dir.c_str(), 0777) == 0) return true;
        if (errno == EEXIST) return true;
        return false;
    }

    void writeLogLine(TactileCollectReal::Imp* imp, const std::string& msg)
    {
        std::cout << msg << std::endl;

        if (imp && imp->log.is_open())
        {
            imp->log << msg << '\n';
            imp->log.flush();
        }
    }

    template<typename... Args>
    void logLine(TactileCollectReal::Imp* imp, Args&&... args)
    {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        writeLogLine(imp, oss.str());
    }

    std::string jsonEscape(const std::string& s)
    {
        std::ostringstream oss;
        for (unsigned char c : s)
        {
            switch (c)
            {
            case '\\': oss << "\\\\"; break;
            case '"':  oss << "\\\""; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default:
                if (c < 0x20)
                {
                    oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c)
                        << std::dec << std::setfill(' ');
                }
                else
                {
                    oss << static_cast<char>(c);
                }
            }
        }
        return oss.str();
    }

    std::string csvEscape(const std::string& s)
    {
        bool need_quote = false;
        for (char c : s)
        {
            if (c == ',' || c == '"' || c == '\n' || c == '\r')
            {
                need_quote = true;
                break;
            }
        }
        if (!need_quote) return s;

        std::string out;
        out.reserve(s.size() + 2);
        out.push_back('"');
        for (char c : s)
        {
            if (c == '"') out += "\"\"";
            else out.push_back(c);
        }
        out.push_back('"');
        return out;
    }

    int getJsonIntValue(const std::string& line, const std::string& key, int default_value)
    {
        const std::string pat = "\"" + key + "\"";
        auto p = line.find(pat);
        if (p == std::string::npos) return default_value;
        p = line.find(':', p + pat.size());
        if (p == std::string::npos) return default_value;
        ++p;
        while (p < line.size() && std::isspace(static_cast<unsigned char>(line[p]))) ++p;
        bool neg = false;
        if (p < line.size() && line[p] == '-') { neg = true; ++p; }
        long v = 0;
        bool any = false;
        while (p < line.size() && std::isdigit(static_cast<unsigned char>(line[p])))
        {
            any = true;
            v = v * 10 + (line[p] - '0');
            ++p;
        }
        if (!any) return default_value;
        return static_cast<int>(neg ? -v : v);
    }

    std::string getJsonStringValue(const std::string& line, const std::string& key, const std::string& default_value)
    {
        const std::string pat = "\"" + key + "\"";
        auto p = line.find(pat);
        if (p == std::string::npos) return default_value;
        p = line.find(':', p + pat.size());
        if (p == std::string::npos) return default_value;
        ++p;
        while (p < line.size() && std::isspace(static_cast<unsigned char>(line[p]))) ++p;
        if (p >= line.size() || line[p] != '"') return default_value;
        ++p;
        std::string out;
        bool esc = false;
        for (; p < line.size(); ++p)
        {
            char c = line[p];
            if (esc)
            {
                switch (c)
                {
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                default: out.push_back(c); break;
                }
                esc = false;
            }
            else if (c == '\\')
            {
                esc = true;
            }
            else if (c == '"')
            {
                return out;
            }
            else
            {
                out.push_back(c);
            }
        }
        return default_value;
    }

    bool jsonAckOk(const std::string& line)
    {
        return line.find("\"ok\":true") != std::string::npos ||
               line.find("\"ok\": true") != std::string::npos;
    }

    void closeTcp(TactileCollectReal::Imp* imp)
    {
        if (!imp) return;
        if (imp->tcp_sock >= 0)
        {
            ::close(imp->tcp_sock);
            imp->tcp_sock = -1;
        }
        imp->tcp_connected = false;
        imp->tcp_request_active = false;
        imp->tcp_rx_buffer.clear();
    }

    bool setNonBlocking(int fd, bool enable)
    {
        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0) return false;
        if (enable) flags |= O_NONBLOCK;
        else flags &= ~O_NONBLOCK;
        return ::fcntl(fd, F_SETFL, flags) == 0;
    }

    bool waitSocketWritable(int fd, int timeout_ms)
    {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd, &wfds);
        timeval tv{};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        const int ret = ::select(fd + 1, nullptr, &wfds, nullptr, &tv);
        return ret > 0 && FD_ISSET(fd, &wfds);
    }

    bool connectTcp(TactileCollectReal::Imp* imp)
    {
        if (!imp || imp->tcp_enable == 0) return true;
        closeTcp(imp);

        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        addrinfo* res = nullptr;
        const std::string port_str = std::to_string(imp->tcp_port);
        const int gai = ::getaddrinfo(imp->tcp_host.c_str(), port_str.c_str(), &hints, &res);
        if (gai != 0)
        {
            logLine(imp, "TCP getaddrinfo failed: host=", imp->tcp_host, " port=", imp->tcp_port, " err=", ::gai_strerror(gai));
            return false;
        }

        bool ok = false;
        for (addrinfo* p = res; p != nullptr; p = p->ai_next)
        {
            int fd = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
            if (fd < 0) continue;

            int one = 1;
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            setNonBlocking(fd, true);

            int ret = ::connect(fd, p->ai_addr, p->ai_addrlen);
            if (ret == 0)
            {
                imp->tcp_sock = fd;
                ok = true;
                break;
            }
            if (errno == EINPROGRESS && waitSocketWritable(fd, imp->tcp_connect_timeout_ms))
            {
                int so_error = 0;
                socklen_t len = sizeof(so_error);
                if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0 && so_error == 0)
                {
                    imp->tcp_sock = fd;
                    ok = true;
                    break;
                }
            }
            ::close(fd);
        }
        ::freeaddrinfo(res);

        if (!ok)
        {
            logLine(imp, "TCP connect failed: ", imp->tcp_host, ":", imp->tcp_port);
            return false;
        }

        imp->tcp_connected = true;
        imp->tcp_rx_buffer.clear();
        logLine(imp, "TCP connected to Windows tactile collector: ", imp->tcp_host, ":", imp->tcp_port);
        return true;
    }

    bool sendTcpLine(TactileCollectReal::Imp* imp, const std::string& line)
    {
        if (!imp || imp->tcp_enable == 0) return true;
        if (!imp->tcp_connected || imp->tcp_sock < 0)
        {
            if (!connectTcp(imp)) return false;
        }

        const std::string payload = line + "\n";
        const char* data = payload.c_str();
        std::size_t left = payload.size();
        while (left > 0)
        {
            ssize_t n = ::send(imp->tcp_sock, data, left, MSG_NOSIGNAL);
            if (n > 0)
            {
                data += n;
                left -= static_cast<std::size_t>(n);
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                if (!waitSocketWritable(imp->tcp_sock, 10)) return false;
                continue;
            }
            logLine(imp, "TCP send failed, errno = ", errno, ". Will close socket.");
            closeTcp(imp);
            return false;
        }
        return true;
    }

    bool pollTcpLine(TactileCollectReal::Imp* imp, std::string& out_line)
    {
        out_line.clear();
        if (!imp || imp->tcp_enable == 0 || !imp->tcp_connected || imp->tcp_sock < 0) return false;

        auto nl = imp->tcp_rx_buffer.find('\n');
        if (nl != std::string::npos)
        {
            out_line = imp->tcp_rx_buffer.substr(0, nl);
            if (!out_line.empty() && out_line.back() == '\r') out_line.pop_back();
            imp->tcp_rx_buffer.erase(0, nl + 1);
            return true;
        }

        char buf[4096];
        while (true)
        {
            ssize_t n = ::recv(imp->tcp_sock, buf, sizeof(buf), MSG_DONTWAIT);
            if (n > 0)
            {
                imp->tcp_rx_buffer.append(buf, buf + n);
                nl = imp->tcp_rx_buffer.find('\n');
                if (nl != std::string::npos)
                {
                    out_line = imp->tcp_rx_buffer.substr(0, nl);
                    if (!out_line.empty() && out_line.back() == '\r') out_line.pop_back();
                    imp->tcp_rx_buffer.erase(0, nl + 1);
                    return true;
                }
                if (imp->tcp_rx_buffer.size() > 1024 * 1024)
                {
                    imp->tcp_rx_buffer.clear();
                    logLine(imp, "TCP receive buffer overflow, cleared.");
                    return false;
                }
                continue;
            }
            if (n == 0)
            {
                logLine(imp, "TCP peer closed connection.");
                closeTcp(imp);
                return false;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
            logLine(imp, "TCP recv failed, errno = ", errno, ". Will close socket.");
            closeTcp(imp);
            return false;
        }
    }

    void drainTcpLines(TactileCollectReal::Imp* imp)
    {
        if (!imp) return;
        std::string line;
        int n = 0;
        while (n < 32 && pollTcpLine(imp, line))
        {
            logLine(imp, "Drain stale TCP line: ", line);
            ++n;
        }
    }

    TactileCollectReal::TactileCollectReal(const std::string& name)
        : imp_(std::make_unique<Imp>())
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"m_tdc\">"
            "  <GroupParam>"
            "    <Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
            "  </GroupParam>"
            "</Command>");
    }

    TactileCollectReal::TactileCollectReal(const TactileCollectReal& other)
        : aris::core::CloneObject<TactileCollectReal, aris::plan::Plan>(other)
        , imp_(std::make_unique<Imp>())
    {
        imp_->m_ = other.imp_->m_;

        imp_->file_ready = false;
        imp_->sample_set_built = other.imp_->sample_set_built;
        imp_->baseline_ready = other.imp_->baseline_ready;

        imp_->stage = other.imp_->stage;
        imp_->stage_wait_count = other.imp_->stage_wait_count;
        imp_->settle_counter = other.imp_->settle_counter;
        imp_->collect_counter = other.imp_->collect_counter;
        imp_->sample_index = other.imp_->sample_index;
        imp_->total_record_count = other.imp_->total_record_count;
        imp_->rot_baseline_stabilize_counter = other.imp_->rot_baseline_stabilize_counter;
        imp_->no_progress_count = other.imp_->no_progress_count;
        imp_->best_pos_err = other.imp_->best_pos_err;
        imp_->best_rot_err = other.imp_->best_rot_err;
        imp_->current_contact_flag = other.imp_->current_contact_flag;

        imp_->baseline_count = other.imp_->baseline_count;
        imp_->baseline_sample_count = other.imp_->baseline_sample_count;

        imp_->csv_dir = other.imp_->csv_dir;
        imp_->csv_path = other.imp_->csv_path;
        // csv stream intentionally not copied/opened here

        std::copy(other.imp_->arm1_p_vector, other.imp_->arm1_p_vector + 9, imp_->arm1_p_vector);
        std::copy(other.imp_->arm1_l_vector, other.imp_->arm1_l_vector + 9, imp_->arm1_l_vector);
        std::copy(other.imp_->arm2_p_vector, other.imp_->arm2_p_vector + 9, imp_->arm2_p_vector);
        std::copy(other.imp_->arm2_l_vector, other.imp_->arm2_l_vector + 9, imp_->arm2_l_vector);

        std::copy(other.imp_->raw_bias_sum, other.imp_->raw_bias_sum + 6, imp_->raw_bias_sum);
        std::copy(other.imp_->raw_bias, other.imp_->raw_bias + 6, imp_->raw_bias);

        for (int i = 0; i < 6; ++i)
        {
            imp_->force_buffer[i] = other.imp_->force_buffer[i];
            imp_->buffer_index[i] = other.imp_->buffer_index[i];
        }

        std::copy(other.imp_->perfect_pose, other.imp_->perfect_pose + 6, imp_->perfect_pose);
        std::copy(other.imp_->init_pose, other.imp_->init_pose + 6, imp_->init_pose);
        std::copy(other.imp_->current_target, other.imp_->current_target + 6, imp_->current_target);
        std::copy(other.imp_->actual_pose, other.imp_->actual_pose + 6, imp_->actual_pose);
        std::copy(other.imp_->transformed_force, other.imp_->transformed_force + 6, imp_->transformed_force);

        std::copy(other.imp_->collect_pose_sum, other.imp_->collect_pose_sum + 6, imp_->collect_pose_sum);
        imp_->collect_pose_valid_count = other.imp_->collect_pose_valid_count;

        imp_->sample_offsets = other.imp_->sample_offsets;
        imp_->sample_names = other.imp_->sample_names;

        imp_->tcp_enable = other.imp_->tcp_enable;
        imp_->tcp_host = other.imp_->tcp_host;
        imp_->tcp_port = other.imp_->tcp_port;
        imp_->tcp_connect_timeout_ms = other.imp_->tcp_connect_timeout_ms;
        imp_->tcp_sample_frames = other.imp_->tcp_sample_frames;
        imp_->tcp_collect_timeout_count = other.imp_->tcp_collect_timeout_count;
        // TCP socket / receive buffer intentionally not copied.
        imp_->tcp_sock = -1;
        imp_->tcp_connected = false;
        imp_->tcp_request_active = false;
        imp_->tcp_request_wait_count = 0;
        imp_->tcp_active_sample_id = -1;
        imp_->tcp_active_record_id = -1;
        imp_->tcp_active_contact_flag = 0;
        imp_->tcp_rx_buffer.clear();
        imp_->tcp_last_ack.clear();
    }

    TactileCollectReal::~TactileCollectReal()
    {
        if (imp_) closeTcp(imp_.get());
        if (imp_ && imp_->csv.is_open())
        {
            imp_->csv.flush();
            imp_->csv.close();
        }
        if (imp_ && imp_->log.is_open())
        {
            imp_->log.flush();
            imp_->log.close();
        }
    }

    auto TactileCollectReal::prepareNrt() -> void
    {
        for (auto& m : motorOptions())
        {
            m = aris::plan::Plan::CHECK_NONE |
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);

        // Fixed TCP configuration. Run the plan directly with `m_tdc`.
        // Windows Python collector must listen on 0.0.0.0:50007.
        imp_->tcp_enable = 1;
        imp_->tcp_host = "192.168.1.7";
        imp_->tcp_port = 50007;
        imp_->tcp_sample_frames = 5;
        imp_->tcp_collect_timeout_count = 6000; // 12 s when robot cycle is 2 ms

        ensureDirExists(imp_->csv_dir);
        const auto timestamp = makeTimestampString();
        imp_->csv_path = imp_->csv_dir + "/gh125_tactile_real_" + timestamp + ".csv";
        imp_->log_path = imp_->csv_dir + "/gh125_tactile_real_" + timestamp + ".txt";
        imp_->csv.open(imp_->csv_path, std::ios::out);
        if (!imp_->csv.is_open())
        {
            throw std::runtime_error("Failed to open csv file for tactile real dataset.");
        }

        imp_->log.open(imp_->log_path, std::ios::out);
        if (!imp_->log.is_open())
        {
            throw std::runtime_error("Failed to open txt log file for tactile real dataset.");
        }

        imp_->csv
            << "global_count,record_id,model,sample_id,sample_name,stage,"
            << "label_dx,label_dy,label_dz,label_rx,label_ry,label_rz,"
            << "target_x,target_y,target_z,target_rx,target_ry,target_rz,"
            << "actual_x,actual_y,actual_z,actual_rx,actual_ry,actual_rz,"
            << "mean_actual_x,mean_actual_y,mean_actual_z,mean_actual_rx,mean_actual_ry,mean_actual_rz,"
            << "real_dx,real_dy,real_dz,real_rx,real_ry,real_rz,"
            << "fx,fy,fz,mx,my,mz,contact_flag,sensor_valid,"
            << "tcp_enable,tcp_ok,tcp_message,tcp_elapsed_ms,tcp_left_frames,tcp_right_frames,tcp_pair_frames,tcp_sample_dir,tcp_ack_raw\n";
        imp_->csv.flush();
        imp_->file_ready = true;

        logLine(imp_.get(), "Open tactile real csv: ", imp_->csv_path);
        logLine(imp_.get(), "Open tactile real txt: ", imp_->log_path);
        logLine(imp_.get(), "TCP config: enable=", imp_->tcp_enable,
            " host=", imp_->tcp_host,
            " port=", imp_->tcp_port,
            " frames=", imp_->tcp_sample_frames,
            " wait_count=", imp_->tcp_collect_timeout_count);
        if (imp_->tcp_enable != 0 && !connectTcp(imp_.get()))
        {
            throw std::runtime_error("Failed to connect Windows tactile TCP collector. Start Python server first or set tcp_enable=0 for robot-only debug.");
        }
    }

    auto TactileCollectReal::executeRT() -> int
    {
        constexpr int max_total_count = 900000;

        // Adaptive stage waiting.
        // 1 count = 2 ms. The normal failure condition is not a fixed 10/15 s timeout.
        // Instead, a movement stage keeps waiting while position/orientation error is
        // still decreasing. It is considered stuck only after no meaningful improvement
        // for no_progress_limit_count. hard_max_stage_wait_count is only a safety cap.
        constexpr int hard_max_stage_wait_count = 30000; // 60 s hard safety upper bound
        constexpr int no_progress_limit_count = 2500;    // 5 s without improvement => stuck
        constexpr double pos_progress_eps = 0.00002;     // 0.02 mm meaningful progress
        constexpr double rot_progress_eps = 0.00005;     // about 0.003 deg meaningful progress

        // Conservative defaults for real machine.
        static double perfect_pose_offset[6]{ 0,0,0,0,0,0 };
        static double preapproach_offset[6]{ 0.000,0.000,0.000,0.000,0.000,0.000 };
        static double contact_bias_offset[6]{ 0.000,0.000,0.000,0.000,0.000,0.000 };
        static int settle_count = 60;
        static int collect_count = 15;
        static int rot_post_retreat_stabilize_count = 300; // 600 ms, used only after rotational samples
        static double pos_tol = 0.0003;
        static double rot_tol = 0.015;

        // The previous robot-only version spent only about 30 ms in COLLECT,
        // while the TCP version must wait about 0.6~0.8 s for Python to save
        // tactile frames. During this longer hold, tiny compliance / force
        // effects can leave a residual baseline error of about 0.5~0.8 mm.
        // Therefore sample-pose judgment remains strict, but RETREAT/RECOVER
        // uses a slightly softer baseline tolerance to avoid stopping the
        // whole 61-sample sequence after a valid tactile sample.
        static double retreat_pos_tol = 0.0012;       // 1.2 mm, used only for returning to baseline
        static double retreat_rot_tol = 0.030;        // about 1.7 deg, used only for returning to baseline
        static double recover_soft_pos_tol = 0.0020;  // 2.0 mm safety accept after no-progress recovery
        static double recover_soft_rot_tol = 0.050;   // about 2.9 deg safety accept after no-progress recovery

        // Contact-force policy for z-axis pressing.
        //   light_contact_force_z: mark light contact in CSV, but continue normal logic.
        //   hold_contact_force_z : stop further pressing and hold current pose for data collection.
        //   danger_contact_force_z: record this sample once, retreat, then continue next sample.
        // Adjust signs/thresholds after checking the transformed fz direction on the real setup.
        static double light_contact_force_z = 2.0;
        static double hold_contact_force_z = 5.0;
        static double danger_contact_force_z = 15.0;
        static double danger_xy_force = 20.0;
        static double danger_moment = 2.0;

        imp_->m_ = int32Param("model");

        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        auto& arm1 = dualArm.subModels().at(0);
        auto& arm2 = dualArm.subModels().at(1);
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        GravComp gc;

        auto clampValue = [&](double v, double lo, double hi)
        {
            return std::max(lo, std::min(hi, v));
        };

        auto poseToRotMatZYX = [&](const double* pose_, double* R)
        {
            const double x = pose_[3];
            const double y = pose_[4];
            const double z = pose_[5];

            const double cx = std::cos(x), sx = std::sin(x);
            const double cy = std::cos(y), sy = std::sin(y);
            const double cz = std::cos(z), sz = std::sin(z);

            R[0] = cz * cy;
            R[1] = cz * sy * sx - sz * cx;
            R[2] = cz * sy * cx + sz * sx;

            R[3] = sz * cy;
            R[4] = sz * sy * sx + cz * cx;
            R[5] = sz * sy * cx - cz * sx;

            R[6] = -sy;
            R[7] = cy * sx;
            R[8] = cy * cx;
        };

        auto calcPosErr = [&](const double* current_pose_, const double* target_pose_) -> double
        {
            const double dx = current_pose_[0] - target_pose_[0];
            const double dy = current_pose_[1] - target_pose_[1];
            const double dz = current_pose_[2] - target_pose_[2];
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        };

        auto calcRotErr = [&](const double* current_pose_, const double* target_pose_) -> double
        {
            double Rc[9]{ 0 }, Rt[9]{ 0 };
            poseToRotMatZYX(current_pose_, Rc);
            poseToRotMatZYX(target_pose_, Rt);

            double Re[9]{ 0 };
            // Re = Rt^T * Rc
            Re[0] = Rt[0] * Rc[0] + Rt[3] * Rc[3] + Rt[6] * Rc[6];
            Re[1] = Rt[0] * Rc[1] + Rt[3] * Rc[4] + Rt[6] * Rc[7];
            Re[2] = Rt[0] * Rc[2] + Rt[3] * Rc[5] + Rt[6] * Rc[8];
            Re[3] = Rt[1] * Rc[0] + Rt[4] * Rc[3] + Rt[7] * Rc[6];
            Re[4] = Rt[1] * Rc[1] + Rt[4] * Rc[4] + Rt[7] * Rc[7];
            Re[5] = Rt[1] * Rc[2] + Rt[4] * Rc[5] + Rt[7] * Rc[8];
            Re[6] = Rt[2] * Rc[0] + Rt[5] * Rc[3] + Rt[8] * Rc[6];
            Re[7] = Rt[2] * Rc[1] + Rt[5] * Rc[4] + Rt[8] * Rc[7];
            Re[8] = Rt[2] * Rc[2] + Rt[5] * Rc[5] + Rt[8] * Rc[8];

            const double trace = Re[0] + Re[4] + Re[8];
            const double cos_theta = clampValue((trace - 1.0) * 0.5, -1.0, 1.0);
            return std::acos(cos_theta);
        };

        // Normalize Euler angle to [-pi, pi].
        // This prevents 359 deg and 0 deg from being treated as far apart.
        auto normalizeAngle = [&](double angle_) -> double
        {
            while (angle_ > PI) angle_ -= 2.0 * PI;
            while (angle_ < -PI) angle_ += 2.0 * PI;
            return angle_;
        };

        // Periodic angular difference: target - current, wrapped to [-pi, pi].
        // Important when rx/ry/rz crosses the 2*pi boundary.
        auto angleDiff = [&](double target_angle_, double current_angle_) -> double
        {
            return normalizeAngle(target_angle_ - current_angle_);
        };

        auto calcRelativeRotVec = [&](const double* ref_pose_, const double* cur_pose_, double* rot_vec)
        {
            double Rr[9]{ 0 }, Rc[9]{ 0 };
            poseToRotMatZYX(ref_pose_, Rr);
            poseToRotMatZYX(cur_pose_, Rc);

            double Re[9]{ 0 };
            // Re = Rr^T * Rc
            Re[0] = Rr[0] * Rc[0] + Rr[3] * Rc[3] + Rr[6] * Rc[6];
            Re[1] = Rr[0] * Rc[1] + Rr[3] * Rc[4] + Rr[6] * Rc[7];
            Re[2] = Rr[0] * Rc[2] + Rr[3] * Rc[5] + Rr[6] * Rc[8];
            Re[3] = Rr[1] * Rc[0] + Rr[4] * Rc[3] + Rr[7] * Rc[6];
            Re[4] = Rr[1] * Rc[1] + Rr[4] * Rc[4] + Rr[7] * Rc[7];
            Re[5] = Rr[1] * Rc[2] + Rr[4] * Rc[5] + Rr[7] * Rc[8];
            Re[6] = Rr[2] * Rc[0] + Rr[5] * Rc[3] + Rr[8] * Rc[6];
            Re[7] = Rr[2] * Rc[1] + Rr[5] * Rc[4] + Rr[8] * Rc[7];
            Re[8] = Rr[2] * Rc[2] + Rr[5] * Rc[5] + Rr[8] * Rc[8];

            const double trace = Re[0] + Re[4] + Re[8];
            const double cos_theta = clampValue((trace - 1.0) * 0.5, -1.0, 1.0);
            const double theta = std::acos(cos_theta);

            if (theta < 1e-9)
            {
                rot_vec[0] = 0.5 * (Re[7] - Re[5]);
                rot_vec[1] = 0.5 * (Re[2] - Re[6]);
                rot_vec[2] = 0.5 * (Re[3] - Re[1]);
                return;
            }

            const double denom = 2.0 * std::sin(theta);
            if (std::fabs(denom) < 1e-9)
            {
                rot_vec[0] = 0.0;
                rot_vec[1] = 0.0;
                rot_vec[2] = 0.0;
                return;
            }

            const double ax = (Re[7] - Re[5]) / denom;
            const double ay = (Re[2] - Re[6]) / denom;
            const double az = (Re[3] - Re[1]) / denom;
            rot_vec[0] = ax * theta;
            rot_vec[1] = ay * theta;
            rot_vec[2] = az * theta;
        };

        auto getRawForceData = [&](double* data_, int m_) -> bool
        {
            int raw_force[6]{ 0 };
            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                {
                    logLine(imp_.get(), "Force Sensor Error");
                    return false;
                }
                data_[i] = static_cast<double>(raw_force[i]) / 1000.0;
            }
            return true;
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];
                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];
                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_)
        {
            for (int i = 0; i < 6; ++i)
            {
                imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;
                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10.0;
            }
        };

        auto getCurrentPoseAndForce = [&](double* pose_, double* transformed_force_, bool* sensor_valid = nullptr)
        {
            double current_angle[12]{ 0 };
            for (int i = 0; i < 12; ++i) current_angle[i] = controller()->motorPool()[i].actualPos();

            dualArm.setInputPos(current_angle);
            if (dualArm.forwardKinematics())
            {
                throw std::runtime_error("Forward Kinematics Failed!");
            }

            double pm[16]{ 0 };
            if (imp_->m_ == 0)
            {
                eeA1.getP(pose_);
                eeA1.getMpm(pm);
            }
            else
            {
                eeA2.getP(pose_);
                eeA2.getMpm(pm);
            }

            double raw_force[6]{ 0 };
            const bool ok = getRawForceData(raw_force, imp_->m_);
            if (sensor_valid) *sensor_valid = ok;
            if (!ok)
            {
                std::fill(transformed_force_, transformed_force_ + 6, 0.0);
                return;
            }

            double bias_removed[6]{ 0 };
            for (int i = 0; i < 6; ++i) bias_removed[i] = raw_force[i] - imp_->raw_bias[i];

            double comp_force[6]{ 0 };
            if (imp_->m_ == 0)
                gc.getCompFT(pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
            else
                gc.getCompFT(pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);

            double actual_force[6]{ 0 };
            for (int i = 0; i < 6; ++i) actual_force[i] = bias_removed[i] + comp_force[i];

            double filtered_force[6]{ 0 };
            forceFilter(actual_force, filtered_force);
            forceTransform(filtered_force, transformed_force_, imp_->m_);
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) -> bool
        {
            double current_angle[12]{ 0 };
            for (int i = 0; i < 12; ++i) current_angle[i] = controller()->motorPool()[i].actualPos();

            double current_joint[6]{ 0 };
            for (int i = 0; i < 6; ++i) current_joint[i] = current_angle[i + 6 * type_];
            model_.setInputPos(current_joint);
            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                logLine(imp_.get(),
                    "Inverse Kinematics Failed | target = [",
                    pos_[0], ", ", pos_[1], ", ", pos_[2], ", ",
                    pos_[3], ", ", pos_[4], ", ", pos_[5], "]");
                return false;
            }

            double x_joint[6]{ 0 };
            model_.getInputPos(x_joint);
            for (std::size_t i = 0; i < 6; ++i)
            {
                controller()->motorPool()[i + 6 * type_].setTargetPos(x_joint[i]);
            }
            return true;
        };

        auto isRotSample = [&](const std::array<double, 6>& sample_offset_) -> bool
        {
            return std::fabs(sample_offset_[3]) > 1e-12 ||
                   std::fabs(sample_offset_[4]) > 1e-12 ||
                   std::fabs(sample_offset_[5]) > 1e-12;
        };

        auto saMoveSmooth = [&](double* desired_pose_, aris::dynamic::Model& model_, int type_, bool is_rot_sample_) -> bool
        {
            double cur_pose[6]{ 0 };
            double cur_force_dummy[6]{ 0 };
            bool sensor_valid = false;
            getCurrentPoseAndForce(cur_pose, cur_force_dummy, &sensor_valid);

            double step_pose[6]{ 0 };
            std::copy(cur_pose, cur_pose + 6, step_pose);

            // Rotational samples are executed more slowly to reduce IK branch jumping
            // and translational coupling during rx/ry/rz perturbations.
            const double pos_step = is_rot_sample_ ? 0.000002 : 0.000010; // m / count
            const double rot_step = is_rot_sample_ ? 0.000025 : 0.000200; // rad / count

            for (int i = 0; i < 3; ++i)
            {
                double d = desired_pose_[i] - cur_pose[i];
                d = std::max(-pos_step, std::min(pos_step, d));
                step_pose[i] = cur_pose[i] + d;
            }
            for (int i = 3; i < 6; ++i)
            {
                // Use periodic angular error instead of direct subtraction.
                // Without this, +1 deg near 359 deg may be interpreted as a full-circle motion.
                double d = angleDiff(desired_pose_[i], cur_pose[i]);
                d = std::max(-rot_step, std::min(rot_step, d));
                step_pose[i] = cur_pose[i] + d;
            }

            return saMove(step_pose, model_, type_);
        };

        auto makeTargetPose = [&](const std::array<double, 6>& sample_offset_, const double* phase_offset_, double* out_pose_)
        {
            for (int i = 0; i < 3; ++i)
            {
                out_pose_[i] = imp_->perfect_pose[i] + sample_offset_[i] + phase_offset_[i];
            }
            for (int i = 3; i < 6; ++i)
            {
                // IMPORTANT:
                // Do NOT normalize the commanded Euler angles here.
                // ARIS and the current robot pose may use a continuous Euler branch, for example
                // rx ~= 6.266 rad and rz ~= 3.205 rad. Normalizing the target to [-pi, pi]
                // gives an equivalent orientation mathematically, but it can make the IK/servo
                // choose an unexpected Cartesian path and cause position drift.
                //
                // The 2*pi crossing problem is handled in saMoveSmooth() and poseReached()
                // by angleDiff()/calcRotErr(), not by changing the target representation.
                out_pose_[i] = imp_->perfect_pose[i] + sample_offset_[i] + phase_offset_[i];
            }
        };

        auto poseReached = [&](const double* current_pose_, const double* target_pose_, const std::array<double, 6>& sample_offset_) -> bool
        {
            const double dx = current_pose_[0] - target_pose_[0];
            const double dy = current_pose_[1] - target_pose_[1];
            const double dz = current_pose_[2] - target_pose_[2];
            const double drx = normalizeAngle(current_pose_[3] - target_pose_[3]);
            const double dry = normalizeAngle(current_pose_[4] - target_pose_[4]);
            const double drz = normalizeAngle(current_pose_[5] - target_pose_[5]);
            const double rot_err = calcRotErr(current_pose_, target_pose_);

            const bool x_cmd = std::fabs(sample_offset_[0]) > 1e-12;
            const bool y_cmd = std::fabs(sample_offset_[1]) > 1e-12;
            const bool z_cmd = std::fabs(sample_offset_[2]) > 1e-12;
            const bool rx_cmd = std::fabs(sample_offset_[3]) > 1e-12;
            const bool ry_cmd = std::fabs(sample_offset_[4]) > 1e-12;
            const bool rz_cmd = std::fabs(sample_offset_[5]) > 1e-12;

            if (!x_cmd && !y_cmd && !z_cmd && !rx_cmd && !ry_cmd && !rz_cmd)
            {
                return calcPosErr(current_pose_, target_pose_) <= pos_tol && rot_err <= rot_tol;
            }

            const bool rot_sample = rx_cmd || ry_cmd || rz_cmd;

            if (rot_sample)
            {
                // Rotational samples are allowed to have translational coupling.
                // The real coupling is still written to CSV as real_dx/real_dy/real_dz.
                // We only reject a rotational sample if the position drift is unsafe or
                // the commanded orientation has not been reached.
                constexpr double rot_sample_max_pos_drift = 0.0040; // 4 mm safety guard
                constexpr double rot_sample_total_rot_tol = 0.0020; // about 0.115 deg; tighter for 1 deg rotational samples
                return (calcPosErr(current_pose_, target_pose_) <= rot_sample_max_pos_drift) &&
                       (rot_err <= rot_sample_total_rot_tol);
            }

            // Translational samples: commanded axes must be tight;
            // non-commanded translational axes allow moderate coupling, especially on z.
            constexpr double cmd_pos_tol = 0.00018;        // 0.18 mm on commanded translational axes
            constexpr double noncmd_xy_tol = 0.00090;     // 0.9 mm on non-commanded x/y axes
            constexpr double noncmd_z_tol = 0.00120;      // 1.2 mm on non-commanded z axis
            constexpr double total_rot_tol = 0.020;       // total orientation error guard

            bool ok = true;
            if (x_cmd) ok = ok && (std::fabs(dx) <= cmd_pos_tol); else ok = ok && (std::fabs(dx) <= noncmd_xy_tol);
            if (y_cmd) ok = ok && (std::fabs(dy) <= cmd_pos_tol); else ok = ok && (std::fabs(dy) <= noncmd_xy_tol);
            if (z_cmd) ok = ok && (std::fabs(dz) <= cmd_pos_tol); else ok = ok && (std::fabs(dz) <= noncmd_z_tol);

            return ok && (rot_err <= total_rot_tol);
        };

        auto baselineReachedForRetreat = [&](const double* current_pose_, const double* target_pose_) -> bool
        {
            return calcPosErr(current_pose_, target_pose_) <= retreat_pos_tol &&
                   calcRotErr(current_pose_, target_pose_) <= retreat_rot_tol;
        };

        auto baselineSoftSafeForRecover = [&](const double* current_pose_, const double* target_pose_) -> bool
        {
            return calcPosErr(current_pose_, target_pose_) <= recover_soft_pos_tol &&
                   calcRotErr(current_pose_, target_pose_) <= recover_soft_rot_tol;
        };

        auto resetWaitMonitor = [&]()
        {
            imp_->stage_wait_count = 0;
            imp_->no_progress_count = 0;
            imp_->best_pos_err = 1.0e100;
            imp_->best_rot_err = 1.0e100;
        };

        auto updateWaitMonitor = [&](const double* current_pose_, const double* target_pose_) -> int
        {
            const double pos_err = calcPosErr(current_pose_, target_pose_);
            const double rot_err = calcRotErr(current_pose_, target_pose_);

            const bool improved =
                (pos_err < imp_->best_pos_err - pos_progress_eps) ||
                (rot_err < imp_->best_rot_err - rot_progress_eps);

            if (improved)
            {
                imp_->best_pos_err = std::min(imp_->best_pos_err, pos_err);
                imp_->best_rot_err = std::min(imp_->best_rot_err, rot_err);
                imp_->no_progress_count = 0;
            }
            else
            {
                ++imp_->no_progress_count;
            }

            ++imp_->stage_wait_count;

            if (imp_->stage_wait_count >= hard_max_stage_wait_count)
            {
                return 2; // hard safety timeout
            }
            if (imp_->no_progress_count >= no_progress_limit_count)
            {
                return 1; // stuck: no meaningful progress
            }
            return 0; // keep waiting
        };

        auto buildSampleSet = [&]()
        {
            if (imp_->sample_set_built) return;
            imp_->sample_offsets.clear();
            imp_->sample_names.clear();

            auto pushSample = [&](const std::array<double, 6>& off, const std::string& name)
            {
                imp_->sample_offsets.push_back(off);
                imp_->sample_names.emplace_back(name);
            };

            // 73-sample set designed for an 8P terminal whose approximate size is:
            // width = 4 mm, length = 13 mm.
            // Convention used here:
            //   x: terminal length direction       -> allow larger translational offsets
            //   y: terminal width direction        -> allow smaller translational offsets
            //   z: insertion/contact depth direction -> most conservative offsets
            // If the real fixture defines x/y differently, swap the values here rather than
            // changing the rest of the state machine.
            constexpr std::array<double, 3> x_levels{ 0.00050, 0.00100, 0.00150 }; // 0.5 / 1.0 / 1.5 mm
            constexpr std::array<double, 3> y_levels{ 0.00025, 0.00050, 0.00075 }; // 0.25 / 0.50 / 0.75 mm
            constexpr std::array<double, 3> z_levels{ 0.00015, 0.00030, 0.00045 }; // 0.15 / 0.30 / 0.45 mm

            // Rotational samples around end-effector x/y/z axes.
            // Unit: rad.
            //   rx: roll around length direction; width half-size is only 2 mm, so 1/2/3 deg are safe.
            //   ry: pitch around width direction; length half-size is 6.5 mm, so use 0.5/1/1.5 deg.
            //   rz: in-plane yaw; 1/2/3 deg gives about 0.11/0.23/0.34 mm edge displacement at half length.
            constexpr std::array<double, 3> rx_levels{ 0.01745329252, 0.03490658504, 0.05235987756 }; // 1 / 2 / 3 deg
            constexpr std::array<double, 3> ry_levels{ 0.00872664626, 0.01745329252, 0.02617993878 }; // 0.5 / 1 / 1.5 deg
            constexpr std::array<double, 3> rz_levels{ 0.01745329252, 0.03490658504, 0.05235987756 }; // 1 / 2 / 3 deg

            // 0) Ideal sample: 1
            pushSample({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, "ideal");

            // 1) Single-axis translational samples: 18
            // x: +/-0.5, +/-1.0, +/-1.5 mm
            for (double dx : x_levels)
            {
                pushSample({ -dx, 0.0, 0.0, 0.0, 0.0, 0.0 }, "dx");
                pushSample({  dx, 0.0, 0.0, 0.0, 0.0, 0.0 }, "dx");
            }
            // y: +/-0.25, +/-0.50, +/-0.75 mm
            for (double dy : y_levels)
            {
                pushSample({ 0.0, -dy, 0.0, 0.0, 0.0, 0.0 }, "dy");
                pushSample({ 0.0,  dy, 0.0, 0.0, 0.0, 0.0 }, "dy");
            }
            // z: +/-0.15, +/-0.30, +/-0.45 mm
            for (double dz : z_levels)
            {
                pushSample({ 0.0, 0.0, -dz, 0.0, 0.0, 0.0 }, "dz");
                pushSample({ 0.0, 0.0,  dz, 0.0, 0.0, 0.0 }, "dz");
            }

            // 2) Coupled translational samples: 36
            // For each level i, use axis-specific magnitudes:
            //   xy = x_levels[i] with y_levels[i]
            //   xz = x_levels[i] with z_levels[i]
            //   yz = y_levels[i] with z_levels[i]
            constexpr std::array<int, 4> signs_a{  1,  1, -1, -1 };
            constexpr std::array<int, 4> signs_b{  1, -1,  1, -1 };
            for (std::size_t level = 0; level < x_levels.size(); ++level)
            {
                const double dx = x_levels[level];
                const double dy = y_levels[level];
                const double dz = z_levels[level];

                for (std::size_t i = 0; i < 4; ++i)
                {
                    pushSample({ signs_a[i] * dx, signs_b[i] * dy, 0.0, 0.0, 0.0, 0.0 }, "xy");
                    pushSample({ signs_a[i] * dx, 0.0, signs_b[i] * dz, 0.0, 0.0, 0.0 }, "xz");
                    pushSample({ 0.0, signs_a[i] * dy, signs_b[i] * dz, 0.0, 0.0, 0.0 }, "yz");
                }
            }

            // 3) Single-axis rotational samples: 18
            // rx: +/-1, +/-2, +/-3 deg
            for (double rx : rx_levels)
            {
                pushSample({ 0.0, 0.0, 0.0, -rx, 0.0, 0.0 }, "rx");
                pushSample({ 0.0, 0.0, 0.0,  rx, 0.0, 0.0 }, "rx");
            }
            // ry: +/-0.5, +/-1.0, +/-1.5 deg
            for (double ry : ry_levels)
            {
                pushSample({ 0.0, 0.0, 0.0, 0.0, -ry, 0.0 }, "ry");
                pushSample({ 0.0, 0.0, 0.0, 0.0,  ry, 0.0 }, "ry");
            }
            // rz: +/-1, +/-2, +/-3 deg
            for (double rz : rz_levels)
            {
                pushSample({ 0.0, 0.0, 0.0, 0.0, 0.0, -rz }, "rz");
                pushSample({ 0.0, 0.0, 0.0, 0.0, 0.0,  rz }, "rz");
            }

            imp_->sample_set_built = true;
            logLine(imp_.get(),
                "Build tactile real 73-sample set for 8P terminal geometry: ideal=1, translation_single=18, translation_coupled=36, rotation_single=18, total sample num = ",
                imp_->sample_offsets.size());
        };

        auto resetCollectMean = [&]()
        {
            std::fill(imp_->collect_pose_sum, imp_->collect_pose_sum + 6, 0.0);
            imp_->collect_pose_valid_count = 0;
        };

        auto accumulateCollectPose = [&](const double* pose_)
        {
            for (int i = 0; i < 6; ++i) imp_->collect_pose_sum[i] += pose_[i];
            ++imp_->collect_pose_valid_count;
        };

        auto getMeanCollectPose = [&](double* mean_pose_)
        {
            if (imp_->collect_pose_valid_count <= 0)
            {
                std::copy(imp_->actual_pose, imp_->actual_pose + 6, mean_pose_);
                return;
            }
            for (int i = 0; i < 6; ++i) mean_pose_[i] = imp_->collect_pose_sum[i] / static_cast<double>(imp_->collect_pose_valid_count);
        };

        auto writeOneRecord = [&](const std::array<double, 6>& sample_offset_,
            const std::string& sample_name_,
            int contact_flag_,
            bool tcp_ok_,
            const std::string& tcp_message_,
            const std::string& tcp_ack_raw_,
            bool accumulate_pose_now_ = true)
        {
            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            if (accumulate_pose_now_) accumulateCollectPose(imp_->actual_pose);

            double mean_pose[6]{ 0 };
            getMeanCollectPose(mean_pose);

            double real_rot_vec[3]{ 0 };
            calcRelativeRotVec(imp_->perfect_pose, mean_pose, real_rot_vec);

            const double real_dx = mean_pose[0] - imp_->perfect_pose[0];
            const double real_dy = mean_pose[1] - imp_->perfect_pose[1];
            const double real_dz = mean_pose[2] - imp_->perfect_pose[2];

            const int ack_elapsed_ms = getJsonIntValue(tcp_ack_raw_, "elapsed_ms", -1);
            const int ack_left_frames = getJsonIntValue(tcp_ack_raw_, "saved_frames_left", -1);
            const int ack_right_frames = getJsonIntValue(tcp_ack_raw_, "saved_frames_right", -1);
            const int ack_pair_frames = getJsonIntValue(tcp_ack_raw_, "saved_pairs", -1);
            const std::string ack_sample_dir = getJsonStringValue(tcp_ack_raw_, "sample_dir", "");

            imp_->csv
                << count() << ','
                << imp_->total_record_count << ','
                << imp_->m_ << ','
                << imp_->sample_index << ','
                << csvEscape(sample_name_) << ','
                << imp_->stage << ','
                << sample_offset_[0] << ',' << sample_offset_[1] << ',' << sample_offset_[2] << ','
                << sample_offset_[3] << ',' << sample_offset_[4] << ',' << sample_offset_[5] << ','
                << imp_->current_target[0] << ',' << imp_->current_target[1] << ',' << imp_->current_target[2] << ','
                << imp_->current_target[3] << ',' << imp_->current_target[4] << ',' << imp_->current_target[5] << ','
                << imp_->actual_pose[0] << ',' << imp_->actual_pose[1] << ',' << imp_->actual_pose[2] << ','
                << imp_->actual_pose[3] << ',' << imp_->actual_pose[4] << ',' << imp_->actual_pose[5] << ','
                << mean_pose[0] << ',' << mean_pose[1] << ',' << mean_pose[2] << ','
                << mean_pose[3] << ',' << mean_pose[4] << ',' << mean_pose[5] << ','
                << real_dx << ',' << real_dy << ',' << real_dz << ','
                << real_rot_vec[0] << ',' << real_rot_vec[1] << ',' << real_rot_vec[2] << ','
                << imp_->transformed_force[0] << ',' << imp_->transformed_force[1] << ',' << imp_->transformed_force[2] << ','
                << imp_->transformed_force[3] << ',' << imp_->transformed_force[4] << ',' << imp_->transformed_force[5] << ','
                << contact_flag_ << ',' << (sensor_valid ? 1 : 0) << ','
                << imp_->tcp_enable << ',' << (tcp_ok_ ? 1 : 0) << ','
                << csvEscape(tcp_message_) << ','
                << ack_elapsed_ms << ',' << ack_left_frames << ',' << ack_right_frames << ',' << ack_pair_frames << ','
                << csvEscape(ack_sample_dir) << ','
                << csvEscape(tcp_ack_raw_)
                << '\n';
            ++imp_->total_record_count;
            imp_->csv.flush();
        };

        auto makeTactileRequestJson = [&](const std::array<double, 6>& sample_offset_, const std::string& sample_name_) -> std::string
        {
            std::ostringstream oss;
            oss << std::setprecision(12);
            oss << "{\"cmd\":\"COLLECT\"";
            oss << ",\"record_id\":" << imp_->total_record_count;
            oss << ",\"model\":" << imp_->m_;
            oss << ",\"sample_id\":" << imp_->sample_index;
            oss << ",\"sample_name\":\"" << jsonEscape(sample_name_) << "\"";
            oss << ",\"frames\":" << imp_->tcp_sample_frames;
            oss << ",\"label_dx\":" << sample_offset_[0];
            oss << ",\"label_dy\":" << sample_offset_[1];
            oss << ",\"label_dz\":" << sample_offset_[2];
            oss << ",\"label_rx\":" << sample_offset_[3];
            oss << ",\"label_ry\":" << sample_offset_[4];
            oss << ",\"label_rz\":" << sample_offset_[5];
            oss << ",\"target_x\":" << imp_->current_target[0];
            oss << ",\"target_y\":" << imp_->current_target[1];
            oss << ",\"target_z\":" << imp_->current_target[2];
            oss << ",\"target_rx\":" << imp_->current_target[3];
            oss << ",\"target_ry\":" << imp_->current_target[4];
            oss << ",\"target_rz\":" << imp_->current_target[5];
            oss << ",\"actual_x\":" << imp_->actual_pose[0];
            oss << ",\"actual_y\":" << imp_->actual_pose[1];
            oss << ",\"actual_z\":" << imp_->actual_pose[2];
            oss << ",\"actual_rx\":" << imp_->actual_pose[3];
            oss << ",\"actual_ry\":" << imp_->actual_pose[4];
            oss << ",\"actual_rz\":" << imp_->actual_pose[5];
            oss << ",\"fx\":" << imp_->transformed_force[0];
            oss << ",\"fy\":" << imp_->transformed_force[1];
            oss << ",\"fz\":" << imp_->transformed_force[2];
            oss << ",\"mx\":" << imp_->transformed_force[3];
            oss << ",\"my\":" << imp_->transformed_force[4];
            oss << ",\"mz\":" << imp_->transformed_force[5];
            oss << ",\"contact_flag\":" << std::max(imp_->current_contact_flag, 1);
            oss << "}";
            return oss.str();
        };

        auto contactFlagFromForce = [&](const double* f_) -> int
        {
            const double abs_fz = std::fabs(f_[2]);
            if (std::fabs(f_[0]) >= danger_xy_force ||
                std::fabs(f_[1]) >= danger_xy_force ||
                abs_fz >= danger_contact_force_z ||
                std::fabs(f_[3]) >= danger_moment ||
                std::fabs(f_[4]) >= danger_moment ||
                std::fabs(f_[5]) >= danger_moment)
            {
                return 3; // danger contact
            }
            if (abs_fz >= hold_contact_force_z) return 2;  // hold-force contact
            if (abs_fz >= light_contact_force_z) return 1; // light contact
            return 0;
        };

        auto updateContactFlag = [&]() -> int
        {
            const int flag = contactFlagFromForce(imp_->transformed_force);
            if (flag > imp_->current_contact_flag) imp_->current_contact_flag = flag;
            return flag;
        };

        auto holdCurrentPoseAndCollect = [&](const std::string& reason_)
        {
            std::copy(imp_->actual_pose, imp_->actual_pose + 6, imp_->current_target);
            imp_->current_contact_flag = std::max(imp_->current_contact_flag, 2);
            logLine(imp_.get(), reason_, " | hold current pose and collect | sample ", imp_->sample_index);
            imp_->stage = STAGE_SETTLE;
            resetWaitMonitor();
            imp_->settle_counter = 0;
            resetCollectMean();
        };

        auto recordDangerAndRetreat = [&](const std::array<double, 6>& sample_offset_, const std::string& sample_name_, const std::string& reason_)
        {
            imp_->current_contact_flag = 3;
            logLine(imp_.get(), reason_, " | danger force = [",
                imp_->transformed_force[0], ", ", imp_->transformed_force[1], ", ", imp_->transformed_force[2], ", ",
                imp_->transformed_force[3], ", ", imp_->transformed_force[4], ", ", imp_->transformed_force[5],
                "] | record once, retreat, then continue next sample ", imp_->sample_index);
            writeOneRecord(sample_offset_, sample_name_, 3, false, "danger_before_tcp_collect", "", true);
            imp_->stage = STAGE_RETREAT;
            resetWaitMonitor();
        };

        auto gotoNextSample = [&]()
        {
            ++imp_->sample_index;
            resetWaitMonitor();
            imp_->settle_counter = 0;
            imp_->collect_counter = 0;
            imp_->rot_baseline_stabilize_counter = 0;
            imp_->current_contact_flag = 0;
            imp_->tcp_request_active = false;
            imp_->tcp_request_wait_count = 0;
            imp_->tcp_active_sample_id = -1;
            imp_->tcp_active_record_id = -1;
            imp_->tcp_active_contact_flag = 0;
            imp_->tcp_last_ack.clear();
            resetCollectMean();
            if (imp_->sample_index >= static_cast<int>(imp_->sample_offsets.size()))
                imp_->stage = STAGE_FINISH;
            else
                imp_->stage = STAGE_MOVE_TO_PREAPPROACH;
        };

        if (count() > max_total_count)
        {
            imp_->csv.flush();
            logLine(imp_.get(), "Tactile real collection timeout | stage = ", imp_->stage, " | sample = ", imp_->sample_index);
            return -3;
        }

        if (imp_->m_ != 0 && imp_->m_ != 1)
        {
            logLine(imp_.get(), "Wrong Model");
            return -2;
        }

        // stage 0: baseline raw force average + lock perfect pose at current pose.
        if (!imp_->baseline_ready)
        {
            double raw_force[6]{ 0 };
            if (!getRawForceData(raw_force, imp_->m_)) return max_total_count - count();
            for (int i = 0; i < 6; ++i) imp_->raw_bias_sum[i] += raw_force[i];
            ++imp_->baseline_count;

            // keep current pose refreshed so operator knows where baseline is captured
            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);

            if (imp_->baseline_count >= imp_->baseline_sample_count)
            {
                for (int i = 0; i < 6; ++i) imp_->raw_bias[i] = imp_->raw_bias_sum[i] / static_cast<double>(imp_->baseline_count);

                std::copy(imp_->actual_pose, imp_->actual_pose + 6, imp_->init_pose);
                for (int i = 0; i < 6; ++i) imp_->perfect_pose[i] = imp_->init_pose[i] + perfect_pose_offset[i];

                buildSampleSet();
                imp_->baseline_ready = true;
                imp_->stage = STAGE_MOVE_TO_PREAPPROACH;
                resetWaitMonitor();

                logLine(imp_.get(), "Baseline Ready");
                logLine(imp_.get(), "Init pose : ", imp_->init_pose[0], "\t", imp_->init_pose[1], "\t", imp_->init_pose[2], "\t", imp_->init_pose[3], "\t", imp_->init_pose[4], "\t", imp_->init_pose[5]);
                logLine(imp_.get(), "Perfect pose : ", imp_->perfect_pose[0], "\t", imp_->perfect_pose[1], "\t", imp_->perfect_pose[2], "\t", imp_->perfect_pose[3], "\t", imp_->perfect_pose[4], "\t", imp_->perfect_pose[5]);
            }
            return max_total_count - count();
        }

        if (imp_->sample_index >= static_cast<int>(imp_->sample_offsets.size()))
        {
            imp_->stage = STAGE_FINISH;
        }

        if (imp_->stage == STAGE_FINISH)
        {
            imp_->csv.flush();
            logLine(imp_.get(), "Tactile real collection finished, total record count = ", imp_->total_record_count);
            return 0;
        }

        const auto& current_offset = imp_->sample_offsets.at(imp_->sample_index);
        const auto& current_name = imp_->sample_names.at(imp_->sample_index);

        if (imp_->stage == STAGE_MOVE_TO_PREAPPROACH)
        {
            if (imp_->stage_wait_count == 0)
            {
                makeTargetPose(current_offset, preapproach_offset, imp_->current_target);
                logLine(imp_.get(), "Try PREAPPROACH | sample ", imp_->sample_index, " : ", current_name, " | offset = [", current_offset[0], ", ", current_offset[1], ", ", current_offset[2], ", ", current_offset[3], ", ", current_offset[4], ", ", current_offset[5], "] | target = [", imp_->current_target[0], ", ", imp_->current_target[1], ", ", imp_->current_target[2], ", ", imp_->current_target[3], ", ", imp_->current_target[4], ", ", imp_->current_target[5], "]");
            }

            if (!saMoveSmooth(imp_->current_target, imp_->m_ == 0 ? model_a1 : model_a2, imp_->m_, isRotSample(current_offset)))
            {
                logLine(imp_.get(), "Skip sample ", imp_->sample_index, " : ", current_name, " | reason = IK failed in preapproach | first retreat to baseline");
                imp_->stage = STAGE_RETREAT;
                resetWaitMonitor();
                return max_total_count - count();
            }

            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            const int pre_force_flag = updateContactFlag();
            if (pre_force_flag >= 3)
            {
                recordDangerAndRetreat(current_offset, current_name, "Danger force in PREAPPROACH");
                return max_total_count - count();
            }
            // In PREAPPROACH, only danger-force protection is active.
            // Hold-force collection is enabled only in CONTACT to avoid false triggering
            // at startup or during free-space preapproach.

            if (poseReached(imp_->actual_pose, imp_->current_target, current_offset))
            {
                logLine(imp_.get(), "Reach preapproach, sample ", imp_->sample_index, " : ", current_name);
                imp_->stage = STAGE_MOVE_TO_CONTACT_SAMPLE;
                resetWaitMonitor();
            }
            else
            {
                const int wait_status = updateWaitMonitor(imp_->actual_pose, imp_->current_target);
                if (imp_->stage_wait_count % 500 == 0)
                {
                    logLine(imp_.get(), "Waiting PREAPPROACH | sample ", imp_->sample_index,
                        " | stage_wait_count = ", imp_->stage_wait_count,
                        " | no_progress_count = ", imp_->no_progress_count,
                        " | best_pos_err = ", imp_->best_pos_err,
                        " | best_rot_err = ", imp_->best_rot_err,
                        " | target = [", imp_->current_target[0], ", ", imp_->current_target[1], ", ", imp_->current_target[2], ", ", imp_->current_target[3], ", ", imp_->current_target[4], ", ", imp_->current_target[5], "] | actual = [", imp_->actual_pose[0], ", ", imp_->actual_pose[1], ", ", imp_->actual_pose[2], ", ", imp_->actual_pose[3], ", ", imp_->actual_pose[4], ", ", imp_->actual_pose[5], "]");
                }
                if (wait_status != 0)
                {
                    logLine(imp_.get(), "Skip sample ", imp_->sample_index, " : ", current_name,
                        " | stage = MOVE_TO_PREAPPROACH | reason = ",
                        (wait_status == 1 ? "preapproach no-progress" : "preapproach hard safety timeout"),
                        " | first retreat to baseline");
                    imp_->stage = STAGE_RETREAT;
                    resetWaitMonitor();
                }
            }
        }
        else if (imp_->stage == STAGE_MOVE_TO_CONTACT_SAMPLE)
        {
            if (imp_->stage_wait_count == 0)
            {
                makeTargetPose(current_offset, contact_bias_offset, imp_->current_target);
                logLine(imp_.get(), "Try CONTACT | sample ", imp_->sample_index, " : ", current_name, " | offset = [", current_offset[0], ", ", current_offset[1], ", ", current_offset[2], ", ", current_offset[3], ", ", current_offset[4], ", ", current_offset[5], "] | target = [", imp_->current_target[0], ", ", imp_->current_target[1], ", ", imp_->current_target[2], ", ", imp_->current_target[3], ", ", imp_->current_target[4], ", ", imp_->current_target[5], "]");
            }

            if (!saMoveSmooth(imp_->current_target, imp_->m_ == 0 ? model_a1 : model_a2, imp_->m_, isRotSample(current_offset)))
            {
                logLine(imp_.get(), "Skip sample ", imp_->sample_index, " : ", current_name, " | reason = IK failed in contact | first retreat to baseline");
                imp_->stage = STAGE_RETREAT;
                resetWaitMonitor();
                return max_total_count - count();
            }

            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            const int contact_force_flag = updateContactFlag();
            if (contact_force_flag >= 3)
            {
                recordDangerAndRetreat(current_offset, current_name, "Danger force in CONTACT");
                return max_total_count - count();
            }
            if (contact_force_flag >= 2)
            {
                holdCurrentPoseAndCollect("Hold-force contact in CONTACT");
                return max_total_count - count();
            }

            if (poseReached(imp_->actual_pose, imp_->current_target, current_offset))
            {
                logLine(imp_.get(), "Reach contact sample pose, sample ", imp_->sample_index);
                imp_->stage = STAGE_SETTLE;
                resetWaitMonitor();
                imp_->settle_counter = 0;
                resetCollectMean();
            }
            else
            {
                const int wait_status = updateWaitMonitor(imp_->actual_pose, imp_->current_target);
                if (imp_->stage_wait_count % 500 == 0)
                {
                    logLine(imp_.get(), "Waiting CONTACT | sample ", imp_->sample_index,
                        " | stage_wait_count = ", imp_->stage_wait_count,
                        " | no_progress_count = ", imp_->no_progress_count,
                        " | pos_err = ", calcPosErr(imp_->actual_pose, imp_->current_target),
                        " | rot_err = ", calcRotErr(imp_->actual_pose, imp_->current_target),
                        " | best_pos_err = ", imp_->best_pos_err,
                        " | best_rot_err = ", imp_->best_rot_err);
                }
                if (wait_status != 0)
                {
                    logLine(imp_.get(), "Skip sample ", imp_->sample_index, " : ", current_name,
                        " | stage = MOVE_TO_CONTACT_SAMPLE | reason = ",
                        (wait_status == 1 ? "contact no-progress" : "contact hard safety timeout"),
                        " | first retreat to baseline");
                    imp_->stage = STAGE_RETREAT;
                    resetWaitMonitor();
                }
            }
        }
        else if (imp_->stage == STAGE_SETTLE)
        {
            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            const int settle_force_flag = updateContactFlag();
            if (settle_force_flag >= 3)
            {
                recordDangerAndRetreat(current_offset, current_name, "Danger force in SETTLE");
                return max_total_count - count();
            }
            ++imp_->settle_counter;
            if (imp_->settle_counter >= settle_count)
            {
                logLine(imp_.get(), "Start collect, sample ", imp_->sample_index, " : ", current_name);
                imp_->stage = STAGE_COLLECT;
                imp_->collect_counter = 0;
                imp_->tcp_request_active = false;
                imp_->tcp_request_wait_count = 0;
                imp_->tcp_active_sample_id = -1;
                imp_->tcp_active_record_id = -1;
                imp_->tcp_active_contact_flag = 0;
                imp_->tcp_last_ack.clear();
                resetCollectMean();
            }
        }
        else if (imp_->stage == STAGE_COLLECT)
        {
            // Keep sampling robot pose/force while Windows collects tactile frames.
            // C++ advances only after Python replies with ACK for the current sample_id.
            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            const int collect_force_flag = updateContactFlag();

            // Keep commanding the sample pose while waiting for Python ACK.
            // This is the main difference from the old robot-only collector:
            // TCP collection is much longer, so we should not leave the robot
            // without a refreshed target command during STAGE_COLLECT.
            if (!saMoveSmooth(imp_->current_target, imp_->m_ == 0 ? model_a1 : model_a2, imp_->m_, isRotSample(current_offset)))
            {
                logLine(imp_.get(), "COLLECT hold IK failed | sample ", imp_->sample_index,
                    " : ", current_name, " | retreat and continue");
                writeOneRecord(current_offset, current_name, std::max(imp_->current_contact_flag, 1), false, "collect_hold_ik_failed", "", false);
                imp_->tcp_request_active = false;
                imp_->tcp_request_wait_count = 0;
                imp_->stage = STAGE_RETREAT;
                resetWaitMonitor();
                return max_total_count - count();
            }

            // Refresh actual pose again after issuing the hold command, then use
            // this pose for the mean real_dx/real_dy/real_dz label.
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            accumulateCollectPose(imp_->actual_pose);

            if (imp_->tcp_enable == 0)
            {
                writeOneRecord(current_offset, current_name, std::max(imp_->current_contact_flag, 1), false, "tcp_disabled_robot_only", "", false);
                ++imp_->collect_counter;
                if (collect_force_flag >= 3 || imp_->collect_counter >= collect_count)
                {
                    imp_->stage = STAGE_RETREAT;
                    resetWaitMonitor();
                }
                return max_total_count - count();
            }

            if (!imp_->tcp_request_active)
            {
                drainTcpLines(imp_.get());
                const std::string req = makeTactileRequestJson(current_offset, current_name);
                if (!sendTcpLine(imp_.get(), req))
                {
                    logLine(imp_.get(), "TCP send COLLECT failed | sample ", imp_->sample_index, " : ", current_name, " | retreat and continue");
                    writeOneRecord(current_offset, current_name, std::max(imp_->current_contact_flag, 1), false, "tcp_send_failed", "", false);
                    imp_->stage = STAGE_RETREAT;
                    resetWaitMonitor();
                    return max_total_count - count();
                }
                imp_->tcp_request_active = true;
                imp_->tcp_request_wait_count = 0;
                imp_->tcp_active_sample_id = imp_->sample_index;
                imp_->tcp_active_record_id = imp_->total_record_count;
                imp_->tcp_active_contact_flag = std::max(imp_->current_contact_flag, 1);
                imp_->tcp_last_ack.clear();
                logLine(imp_.get(), "TCP COLLECT sent | sample ", imp_->sample_index, " : ", current_name,
                    " | frames = ", imp_->tcp_sample_frames);
            }

            std::string ack_line;
            if (pollTcpLine(imp_.get(), ack_line))
            {
                const int ack_sample_id = getJsonIntValue(ack_line, "sample_id", -999999);
                if (ack_sample_id != imp_->tcp_active_sample_id)
                {
                    logLine(imp_.get(), "Ignore stale TCP ACK | expected sample_id=", imp_->tcp_active_sample_id,
                        " | got line=", ack_line);
                    return max_total_count - count();
                }

                const bool ok = jsonAckOk(ack_line);
                imp_->tcp_last_ack = ack_line;
                logLine(imp_.get(), ok ? "TCP ACK OK | " : "TCP ACK NACK | ", ack_line);
                writeOneRecord(current_offset, current_name, std::max(imp_->current_contact_flag, 1), ok, ok ? "tcp_ack_ok" : "tcp_ack_nack", ack_line, false);
                imp_->tcp_request_active = false;
                imp_->tcp_request_wait_count = 0;
                imp_->stage = STAGE_RETREAT;
                resetWaitMonitor();
                return max_total_count - count();
            }

            ++imp_->tcp_request_wait_count;
            if (imp_->tcp_request_wait_count % 500 == 0)
            {
                logLine(imp_.get(), "Waiting TCP ACK | sample ", imp_->sample_index,
                    " | tcp_request_wait_count = ", imp_->tcp_request_wait_count,
                    " | force_flag = ", collect_force_flag);
            }
            if (imp_->tcp_request_wait_count >= imp_->tcp_collect_timeout_count)
            {
                logLine(imp_.get(), "TCP ACK timeout | sample ", imp_->sample_index,
                    " | wait_count = ", imp_->tcp_request_wait_count,
                    " | retreat and continue");
                writeOneRecord(current_offset, current_name, std::max(imp_->current_contact_flag, 1), false, "tcp_ack_timeout", "", false);
                imp_->tcp_request_active = false;
                imp_->tcp_request_wait_count = 0;
                imp_->stage = STAGE_RETREAT;
                resetWaitMonitor();
                return max_total_count - count();
            }
        }
        else if (imp_->stage == STAGE_RETREAT)
        {
            if (imp_->stage_wait_count == 0)
            {
                const std::array<double, 6> retreat_offset{ 0.0,0.0,0.0,0.0,0.0,0.0 };
                makeTargetPose(retreat_offset, preapproach_offset, imp_->current_target);
                logLine(imp_.get(), "Try RETREAT | sample ", imp_->sample_index, " : ", current_name, " | offset = [", current_offset[0], ", ", current_offset[1], ", ", current_offset[2], ", ", current_offset[3], ", ", current_offset[4], ", ", current_offset[5], "] | target = [", imp_->current_target[0], ", ", imp_->current_target[1], ", ", imp_->current_target[2], ", ", imp_->current_target[3], ", ", imp_->current_target[4], ", ", imp_->current_target[5], "]");
            }

            if (!saMoveSmooth(imp_->current_target, imp_->m_ == 0 ? model_a1 : model_a2, imp_->m_, false))
            {
                logLine(imp_.get(), "Retreat IK failed | enter RECOVER_TO_BASELINE and do not advance sample.");
                imp_->stage = STAGE_RECOVER_TO_BASELINE;
                resetWaitMonitor();
                return max_total_count - count();
            }

            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            const std::array<double, 6> retreat_offset{ 0.0,0.0,0.0,0.0,0.0,0.0 };
            if (baselineReachedForRetreat(imp_->actual_pose, imp_->current_target))
            {
                if (isRotSample(current_offset))
                {
                    logLine(imp_.get(), "Retreat complete after rotational sample, stabilize at baseline before next sample = ", (imp_->sample_index + 1));
                    imp_->stage = STAGE_ROT_BASELINE_STABILIZE;
                    imp_->rot_baseline_stabilize_counter = 0;
                    resetWaitMonitor();
                }
                else
                {
                    logLine(imp_.get(), "Retreat complete, next sample = ", (imp_->sample_index + 1));
                    gotoNextSample();
                }
            }
            else
            {
                const int wait_status = updateWaitMonitor(imp_->actual_pose, imp_->current_target);
                if (imp_->stage_wait_count % 500 == 0)
                {
                    logLine(imp_.get(), "Waiting RETREAT | sample ", imp_->sample_index,
                        " | stage_wait_count = ", imp_->stage_wait_count,
                        " | no_progress_count = ", imp_->no_progress_count,
                        " | pos_err = ", calcPosErr(imp_->actual_pose, imp_->current_target),
                        " | rot_err = ", calcRotErr(imp_->actual_pose, imp_->current_target),
                        " | best_pos_err = ", imp_->best_pos_err,
                        " | best_rot_err = ", imp_->best_rot_err);
                }
                if (wait_status != 0)
                {
                    logLine(imp_.get(), "RETREAT failed | sample ", imp_->sample_index, " : ", current_name,
                        " | reason = ",
                        (wait_status == 1 ? "retreat no-progress" : "retreat hard safety timeout"),
                        " | enter RECOVER_TO_BASELINE; do not advance sample.");
                    imp_->stage = STAGE_RECOVER_TO_BASELINE;
                    resetWaitMonitor();
                }
            }
        }
        else if (imp_->stage == STAGE_ROT_BASELINE_STABILIZE)
        {
            // Hold the baseline pose for a short time after a rotational sample.
            // The hold counter only advances when the robot is actually within the strict baseline tolerance.
            const std::array<double, 6> retreat_offset{ 0.0,0.0,0.0,0.0,0.0,0.0 };
            makeTargetPose(retreat_offset, preapproach_offset, imp_->current_target);
            if (!saMoveSmooth(imp_->current_target, imp_->m_ == 0 ? model_a1 : model_a2, imp_->m_, false))
            {
                logLine(imp_.get(), "Baseline stabilization IK failed after rotational sample | enter RECOVER_TO_BASELINE.");
                imp_->stage = STAGE_RECOVER_TO_BASELINE;
                resetWaitMonitor();
                return max_total_count - count();
            }

            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            if (baselineReachedForRetreat(imp_->actual_pose, imp_->current_target))
            {
                ++imp_->rot_baseline_stabilize_counter;
                if (imp_->rot_baseline_stabilize_counter >= rot_post_retreat_stabilize_count)
                {
                    logLine(imp_.get(), "Rotational baseline stabilization complete, next sample = ", (imp_->sample_index + 1));
                    gotoNextSample();
                }
            }
            else
            {
                imp_->rot_baseline_stabilize_counter = 0;
                const int wait_status = updateWaitMonitor(imp_->actual_pose, imp_->current_target);
                if (imp_->stage_wait_count % 500 == 0)
                {
                    logLine(imp_.get(), "Waiting ROT_BASELINE_STABILIZE | sample ", imp_->sample_index,
                        " | stage_wait_count = ", imp_->stage_wait_count,
                        " | no_progress_count = ", imp_->no_progress_count,
                        " | pos_err = ", calcPosErr(imp_->actual_pose, imp_->current_target),
                        " | rot_err = ", calcRotErr(imp_->actual_pose, imp_->current_target));
                }
                if (wait_status != 0)
                {
                    logLine(imp_.get(), "Rotational baseline stabilization failed | enter RECOVER_TO_BASELINE; do not advance sample.");
                    imp_->stage = STAGE_RECOVER_TO_BASELINE;
                    resetWaitMonitor();
                }
            }
        }
        else if (imp_->stage == STAGE_RECOVER_TO_BASELINE)
        {
            // Recovery is stricter than sample collection.
            // It is used when RETREAT or post-rotation stabilization cannot verify the baseline.
            // The program must never continue to the next sample from an unverified baseline,
            // otherwise pose error accumulates and the robot may drift far away.
            if (imp_->stage_wait_count == 0)
            {
                const std::array<double, 6> retreat_offset{ 0.0,0.0,0.0,0.0,0.0,0.0 };
                makeTargetPose(retreat_offset, preapproach_offset, imp_->current_target);
                logLine(imp_.get(), "Try RECOVER_TO_BASELINE | sample ", imp_->sample_index,
                    " : ", current_name,
                    " | target = [", imp_->current_target[0], ", ", imp_->current_target[1], ", ", imp_->current_target[2], ", ",
                    imp_->current_target[3], ", ", imp_->current_target[4], ", ", imp_->current_target[5], "]");
            }

            if (!saMoveSmooth(imp_->current_target, imp_->m_ == 0 ? model_a1 : model_a2, imp_->m_, false))
            {
                logLine(imp_.get(), "RECOVER_TO_BASELINE IK failed | stop plan to prevent accumulated drift.");
                imp_->csv.flush();
                return -5;
            }

            bool sensor_valid = false;
            getCurrentPoseAndForce(imp_->actual_pose, imp_->transformed_force, &sensor_valid);
            const std::array<double, 6> retreat_offset{ 0.0,0.0,0.0,0.0,0.0,0.0 };
            if (baselineReachedForRetreat(imp_->actual_pose, imp_->current_target))
            {
                logLine(imp_.get(), "RECOVER_TO_BASELINE complete, next sample = ", (imp_->sample_index + 1));
                if (isRotSample(current_offset))
                {
                    imp_->stage = STAGE_ROT_BASELINE_STABILIZE;
                    imp_->rot_baseline_stabilize_counter = 0;
                    resetWaitMonitor();
                }
                else
                {
                    gotoNextSample();
                }
            }
            else
            {
                const int wait_status = updateWaitMonitor(imp_->actual_pose, imp_->current_target);
                if (imp_->stage_wait_count % 500 == 0)
                {
                    logLine(imp_.get(), "Waiting RECOVER_TO_BASELINE | sample ", imp_->sample_index,
                        " | stage_wait_count = ", imp_->stage_wait_count,
                        " | no_progress_count = ", imp_->no_progress_count,
                        " | pos_err = ", calcPosErr(imp_->actual_pose, imp_->current_target),
                        " | rot_err = ", calcRotErr(imp_->actual_pose, imp_->current_target),
                        " | best_pos_err = ", imp_->best_pos_err,
                        " | best_rot_err = ", imp_->best_rot_err);
                }
                if (wait_status != 0)
                {
                    if (baselineSoftSafeForRecover(imp_->actual_pose, imp_->current_target))
                    {
                        logLine(imp_.get(), "RECOVER_TO_BASELINE soft accepted | reason = ",
                            (wait_status == 1 ? "recover no-progress" : "recover hard safety timeout"),
                            " | pos_err = ", calcPosErr(imp_->actual_pose, imp_->current_target),
                            " | rot_err = ", calcRotErr(imp_->actual_pose, imp_->current_target),
                            " | continue next sample to avoid stopping after a valid tactile sample.");
                        gotoNextSample();
                    }
                    else
                    {
                        logLine(imp_.get(), "RECOVER_TO_BASELINE failed | reason = ",
                            (wait_status == 1 ? "recover no-progress" : "recover hard safety timeout"),
                            " | stop plan to prevent accumulated drift. Current actual = [",
                            imp_->actual_pose[0], ", ", imp_->actual_pose[1], ", ", imp_->actual_pose[2], ", ",
                            imp_->actual_pose[3], ", ", imp_->actual_pose[4], ", ", imp_->actual_pose[5], "]",
                            " | pos_err = ", calcPosErr(imp_->actual_pose, imp_->current_target),
                            " | rot_err = ", calcRotErr(imp_->actual_pose, imp_->current_target));
                        imp_->csv.flush();
                        return -5;
                    }
                }
            }
        }
        else if (imp_->stage == STAGE_FINISH)
        {
            imp_->csv.flush();
            logLine(imp_.get(), "Tactile real collection finished, total record count = ", imp_->total_record_count);
            return 0;
        }

        return max_total_count - count();
    }
}


ARIS_REGISTRATION
{
    aris::core::class_<tactile_collect_real::TactileCollectReal>("TactileCollectReal")
        .inherit<aris::plan::Plan>();
}