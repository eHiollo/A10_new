#pragma once
#include <memory>
#include <filesystem>
#include "aris.hpp"
#include "kaanh/follower/udpFollower.hpp"
#include "a10_follower_core.hpp"

namespace a10_follower {
void set_config_directory(const std::filesystem::path& directory);
void request_stop() noexcept;

class A10UdpFollower : public kaanh::follower::UDPFollower {
public:
    A10UdpFollower();
    ~A10UdpFollower();
    void init() override;
    auto execute(const std::string&, std::function<void(std::string)>) noexcept
        -> std::pair<std::string, std::string> override;
    void receive(const std::string& body) noexcept;
    nlohmann::json diagnostics() const;
    void configure(const std::string& command, bool allow_save = true);
    std::uint64_t begin(const std::string& command, const Joints& minimum, const Joints& maximum);
    bool start_rt(std::uint64_t session) noexcept;
    bool stop_requested() const noexcept;
    bool read_target_rt(std::uint64_t session, Joints& target, std::uint64_t& revision) noexcept;
    bool read_limits_rt(Limits& limits, std::uint64_t& revision) noexcept;
    void publish_rt(bool stopping, bool settled, const char* fault) noexcept;
    void finish(std::uint64_t session) noexcept;
    static A10UdpFollower& instance();
private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

class A10JointFollower : public aris::core::CloneObject<A10JointFollower, aris::plan::Plan> {
public:
    explicit A10JointFollower(const std::string& name = "A10JointFollower");
    ~A10JointFollower();
    void prepareNrt() override;
    int executeRT() override;
    void collectNrt() override;
    KAANH_DECLARE_BIG_FOUR(A10JointFollower)
private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;
};
} // namespace a10_follower
