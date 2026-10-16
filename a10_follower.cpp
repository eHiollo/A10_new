#include "a10_follower.hpp"
#include "a10_follower_protocol.hpp"
#include "a10_motion_lease.hpp"
#include "kaanh/middleware/shell.hpp"
#include "kaanh/robot/robot.hpp"
#include <chrono>
#include <iostream>
#include <mutex>

namespace a10_follower {
namespace {
enum class Phase { Idle, Starting, Running, Stopping, Faulted };
std::filesystem::path config_directory;
std::atomic<A10UdpFollower*> module{nullptr};
std::uint64_t now_ns() noexcept {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
const char* phase_name(Phase phase) {
    switch (phase) {
    case Phase::Idle: return "Stopped";
    case Phase::Starting: return "Starting";
    case Phase::Running: return "Running";
    case Phase::Stopping: return "Stopping";
    default: return "Faulted";
    }
}
void require_ready(aris::server::ControlServer& cs) {
    auto& robot = kaanh::robot::Robot::instanceInCs();
    if (cs.errorCode() != 0 || robot.robotStatus() != kaanh::robot::RobotStatus::kNormal)
        throw std::runtime_error("follower requires fault-free controller");
    if (robot.robotActivate() != kaanh::robot::RobotActivate::kEnabled)
        throw std::runtime_error("follower requires enabled robot; it never enables motors");
    if (robot.robotMotion() != kaanh::robot::RobotMotion::kStopped || (cs.running() && cs.currentExecutePlan()))
        throw std::runtime_error("follower requires robot motion Stopped and no active plan");
    auto& motors = cs.controller().motorPool();
    if (motors.size() < 6) throw std::runtime_error("follower requires six real motors");
    for (std::size_t i = 0; i < 6; ++i) {
        auto* motor = dynamic_cast<aris::control::EthercatMotor*>(&motors[i]);
        if (!motor || !motor->slave() || motor->slave()->isVirtual() || !motor->isEnabled()
            || (motor->statusWord() & 8U) || !std::isfinite(motor->actualPos())
            || !std::isfinite(motor->actualVel()) || std::abs(motor->actualVel()) > 0.03
            || std::abs(motor->actualPos() - motor->targetPos()) > 0.01)
            throw std::runtime_error("follower requires enabled, stationary, real six-axis feedback");
    }
}
}
void set_config_directory(const std::filesystem::path& path) { config_directory = path; }
struct A10UdpFollower::Imp {
    mutable std::mutex config_mutex;
    std::mutex input_mutex;
    Limits configured;
    Mailbox<3> limits;
    Mailbox<6> target;
    Joints minimum{}, maximum{};
    std::atomic<Phase> phase{Phase::Idle};
    std::atomic<std::uint64_t> session{0}, received{0}, rejected{0}, ignored{0}, last_target_ns{0}, applied_revision{0};
    std::atomic<bool> stop{false}, settled{true};
    std::atomic<const char*> fault{"none"};
    std::string config_error;
    std::filesystem::path path;
};
A10UdpFollower::A10UdpFollower() : imp_(new Imp) {
    setName("UDPFollower");
    if (!config_directory.empty()) imp_->path = config_directory / "follower_config.json";
    imp_->limits.publish(imp_->configured.values());
}
A10UdpFollower::~A10UdpFollower() {
    // Stop receivers before destroying state captured by their callbacks.
    for (auto& param : followerParamPool()) param.socketServer().stop();
    auto* expected = this; module.compare_exchange_strong(expected, nullptr);
}
A10UdpFollower& A10UdpFollower::instance() {
    auto* result = module.load();
    if (!result) throw std::runtime_error("A10UdpFollower not initialized; migrate follower XML");
    return *result;
}
void A10UdpFollower::init() {
    kaanh::follower::UDPFollower::init();
    if (config_directory.empty()) throw std::runtime_error("follower configuration directory is not set");
    imp_->path = config_directory / "follower_config.json";
    for (auto& plan : aris::server::ControlServer::instance().planRoot().planPool())
        if (dynamic_cast<A10JointFollower*>(&plan)) {
            imp_->configured.max_vel = std::stod(plan.command().findParam("max_vel")->defaultValue());
            imp_->configured.max_acc = std::stod(plan.command().findParam("max_acc")->defaultValue());
            imp_->configured.max_dec = std::stod(plan.command().findParam("max_dec")->defaultValue());
        }
    try { imp_->configured = load_limits(imp_->path, imp_->configured); }
    catch (const std::exception& error) {
        imp_->config_error = error.what();
        std::cerr << "follower configuration invalid: " << error.what() << "; follower_aj disabled until corrected" << std::endl;
    }
    imp_->limits.publish(imp_->configured.values());
    bool replaced = false;
    for (auto& param : followerParamPool())
        if (dynamic_cast<kaanh::follower::FollowerAjParam*>(&param)) {
            param.socketServer().setOnReceivedMsg([this](aris::core::SocketServer*, int, aris::core::Msg& msg) {
                receive(msg.toString()); return 0;
            });
            replaced = true;
        }
    if (!replaced) throw std::runtime_error("FollowerAjParam missing from UDPFollower XML");
    module.store(this);
    kaanh::robot::Robot::instanceInCs().registerGetFunc(
        [this](aris::server::ControlServer&) { return diagnostics(); }, "follower_control", kaanh::robot::Robot::GetType::kInfo);
}
void request_stop() noexcept {
    if (auto* value = module.load()) {
        // Use the same command-independent stop path as stop_follower.
        value->publish_rt(true, false, "none");
    }
}
void A10UdpFollower::configure(const std::string& command, bool allow_save) {
    std::lock_guard<std::mutex> lock(imp_->config_mutex);
    if (!imp_->config_error.empty()) throw std::runtime_error("invalid saved/default follower configuration: " + imp_->config_error);
    auto update = parse_config(command, imp_->configured, allow_save);
    if (update.save) {
        if (imp_->path.empty()) throw std::runtime_error("follower configuration directory not set");
        save_limits(imp_->path, update.limits);
    }
    if (update.changed) {
        imp_->configured = update.limits;
        imp_->limits.publish(update.limits.values());
    }
}
auto A10UdpFollower::execute(const std::string& body, std::function<void(std::string)> reply) noexcept
    -> std::pair<std::string,std::string> {
    const auto command = getCmdName(body);
    try {
        if (command == "follower_aj") {
            // Submit directly to ControlServer, bypassing Shell's completion
            // callback. This long-lived Plan must acknowledge preparation now,
            // not hold the caller's WebSocket until stop/collect.
            auto plan = aris::server::ControlServer::instance().executeCmd(body);
            if (!plan) throw std::runtime_error("follower command was not prepared");
            if (plan->prepareRetCode() != 0) {
                sendRet(reply, plan->prepareRetCode(), plan->prepareRetMsg()); return {"", ""};
            }
            sendRet(reply); return {"", ""};
        }
        if (command == "follower_config") {
            configure(body);
            sendRet(reply, 0, "", diagnostics().dump()); return {"", ""};
        }
        if (command == "stop_follower") {
            if (imp_->phase.load() != Phase::Idle && imp_->phase.load() != Phase::Faulted) {
                publish_rt(true, false, "none"); sendRet(reply); return {"", ""};
            }
            // Preserve the SDK cartesian stop path; idle joint stops are idempotent.
            if (!isRunning()) { sendRet(reply); return {"", ""}; }
        }
        if (command == "start_follower" && motion_owner.load() != MotionOwner::None)
            throw std::runtime_error("a motion controller is already active");
        return kaanh::follower::UDPFollower::execute(body, std::move(reply));
    } catch (const std::exception& error) { sendRet(reply, -1, error.what()); return {"", ""}; }
    catch (...) { sendRet(reply, -1, "follower command failed"); return {"", ""}; }
}
void A10UdpFollower::receive(const std::string& body) noexcept {
    try {
        const auto joints = parse_jp(body);
        std::lock_guard<std::mutex> lock(imp_->input_mutex);
        if (imp_->phase.load() != Phase::Running || imp_->stop.load()) { ++imp_->ignored; return; }
        for (std::size_t i = 0; i < 6; ++i)
            if (joints[i] <= imp_->minimum[i] + 0.002 || joints[i] >= imp_->maximum[i] - 0.002) {
                ++imp_->rejected; return;
            }
        const auto stamp = now_ns();
        imp_->target.publish(joints, imp_->session.load(), stamp);
        imp_->last_target_ns.store(stamp); ++imp_->received;
    } catch (...) { ++imp_->rejected; }
}
std::uint64_t A10UdpFollower::begin(const std::string& command, const Joints& minimum, const Joints& maximum) {
    std::scoped_lock lock(imp_->config_mutex, imp_->input_mutex);
    if (!imp_->config_error.empty()) throw std::runtime_error("invalid follower configuration: " + imp_->config_error);
    if (imp_->phase.load() != Phase::Idle && imp_->phase.load() != Phase::Faulted)
        throw std::runtime_error("follower already starting/running/stopping");
    auto update = parse_config(command, imp_->configured, false);
    if (update.changed) { imp_->configured = update.limits; imp_->limits.publish(update.limits.values()); }
    imp_->minimum = minimum; imp_->maximum = maximum;
    imp_->stop.store(false); imp_->settled.store(true); imp_->fault.store("none"); imp_->last_target_ns.store(0);
    const auto session = imp_->session.fetch_add(1) + 1;
    // This SDK setter uses a mutex, so mode changes belong exclusively in NRT.
    setFollowerMode(kaanh::follower::FollowerMode::kAixs);
    imp_->phase.store(Phase::Starting); return session;
}
bool A10UdpFollower::start_rt(std::uint64_t session) noexcept {
    if (session != imp_->session.load() || imp_->stop.load()) return false;
    imp_->phase.store(Phase::Running);
    setRunningState(true); return true; // SDK running flag is atomic.
}
bool A10UdpFollower::stop_requested() const noexcept { return imp_->stop.load(); }
bool A10UdpFollower::read_target_rt(std::uint64_t session, Joints& target, std::uint64_t& revision) noexcept {
    Mailbox<6>::Snapshot value;
    if (!imp_->target.read(value) || value.token != session || value.version == revision) return false;
    target = value.values; revision = value.version; return true;
}
bool A10UdpFollower::read_limits_rt(Limits& limits, std::uint64_t& revision) noexcept {
    Mailbox<3>::Snapshot value;
    if (!imp_->limits.read(value)) return false;
    limits = {value.values[0],value.values[1],value.values[2]}; revision = value.version;
    imp_->applied_revision.store(revision); return true;
}
void A10UdpFollower::publish_rt(bool stopping, bool settled, const char* fault) noexcept {
    const auto phase = imp_->phase.load();
    if (phase == Phase::Idle || phase == Phase::Faulted) return;
    if (fault && std::string_view(fault) != "none") imp_->fault.store(fault);
    if (stopping) { imp_->stop.store(true); imp_->phase.store(Phase::Stopping); }
    imp_->settled.store(settled);
}
void A10UdpFollower::finish(std::uint64_t session) noexcept {
    if (session != imp_->session.load()) return;
    imp_->stop.store(true);
    setRunningState(false);
    setFollowerMode(kaanh::follower::FollowerMode::kNone);
    imp_->phase.store(std::string_view(imp_->fault.load()) == "none" ? Phase::Idle : Phase::Faulted);
}
nlohmann::json A10UdpFollower::diagnostics() const {
    std::lock_guard<std::mutex> lock(imp_->config_mutex);
    auto json = limits_json(imp_->configured);
    Joints caps{}; for (std::size_t i=0;i<6;++i) caps[i]=std::min(imp_->configured.max_vel,Guard::speed_limit(i));
    Mailbox<3>::Snapshot config; imp_->limits.read(config);
    const auto stamp=imp_->last_target_ns.load();
    json.update({{"state",phase_name(imp_->phase.load())},{"config_version",config.version},
        {"applied_config_version",imp_->applied_revision.load()}, {"effective_max_vel",caps},
        {"session",imp_->session.load()}, {"settled",imp_->settled.load()},
        {"received",imp_->received.load()}, {"rejected",imp_->rejected.load()}, {"ignored",imp_->ignored.load()},
        {"target_age_s",stamp ? static_cast<double>(now_ns()-stamp)*1e-9 : -1.0},
        {"fault",imp_->fault.load()}, {"config_error",imp_->config_error}, {"units","rad/s, rad/s^2"}});
    return json;
}

struct A10JointFollower::Imp {
    Tracker tracker;
    Limits limits;
    A10UdpFollower* module{nullptr};
    std::uint64_t session{0}, target_revision{0}, config_revision{0};
    std::shared_ptr<MotionLease> lease;
};
A10JointFollower::A10JointFollower(const std::string&) : imp_(new Imp) {
    aris::core::fromXmlString(command(), "<Command name=\"follower_aj\"><GroupParam name=\"group_param\">"
        "<Param name=\"max_vel\" default=\"0.3\"/><Param name=\"max_acc\" default=\"1.5\"/>"
        "<Param name=\"max_dec\" default=\"1.5\"/></GroupParam></Command>");
}
A10JointFollower::~A10JointFollower() = default;
void A10JointFollower::prepareNrt() {
    auto lease=MotionLease::acquire(MotionOwner::Follower);
    auto& cs=aris::server::ControlServer::instance(); require_ready(cs);
    Joints actual{}, command{}, minimum{}, maximum{};
    auto& motors=controller()->motorPool();
    for(std::size_t i=0;i<6;++i) {
        actual[i]=motors[i].actualPos(); command[i]=motors[i].targetPos();
        minimum[i]=motors[i].minPos(); maximum[i]=motors[i].maxPos();
    }
    if(!imp_->tracker.reset(command,actual,minimum,maximum)) throw std::runtime_error("invalid initial follower joint state/limits");
    imp_->module=&A10UdpFollower::instance();
    imp_->session=imp_->module->begin(std::string(cmdString()),minimum,maximum);
    imp_->target_revision=0; imp_->config_revision=0;
    // Keep SDK enable/position/following checks; the independent guard adds
    // bounded command acceleration and an earlier 0.1 rad following limit.
    for(auto& value:motorOptions()) value |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    imp_->lease=std::move(lease);
}
int A10JointFollower::executeRT() {
    auto& module=*imp_->module;
    auto& motors=controller()->motorPool();
    Joints actual{};
    if(count()==1) {
        Joints command{}, minimum{}, maximum{};
        bool valid=true;
        for (std::size_t i=0;i<6;++i) {
            actual[i]=motors[i].actualPos(); command[i]=motors[i].targetPos();
            minimum[i]=motors[i].minPos(); maximum[i]=motors[i].maxPos();
            valid = valid && motors[i].isEnabled() && !(motors[i].statusWord() & 8U)
                && std::isfinite(motors[i].actualVel()) && std::abs(motors[i].actualVel())<=0.03;
        }
        if(!valid || !imp_->tracker.reset(command,actual,minimum,maximum)) {
            module.publish_rt(true,false,"invalid_initial_feedback"); return 0;
        }
        if(!module.start_rt(imp_->session)) return 0;
    }
    bool physical_stop=true, at_command=true;
    for(std::size_t i=0;i<6;++i) {
        actual[i]=motors[i].actualPos();
        if(!std::isfinite(motors[i].actualVel())) imp_->tracker.fault("non_finite_joint_velocity");
        if(!motors[i].isEnabled() || (motors[i].statusWord() & 8U)) imp_->tracker.fault("motor_disabled_or_fault");
        if(!std::isfinite(motors[i].actualVel()) || std::abs(motors[i].actualVel())>0.03
            || !std::isfinite(actual[i])) physical_stop=false;
        if(!std::isfinite(actual[i]) || std::abs(actual[i]-imp_->tracker.position()[i])>0.01) at_command=false;
    }
    module.read_limits_rt(imp_->limits,imp_->config_revision);
    const bool stopping=module.stop_requested() || imp_->tracker.faulted();
    if(!stopping) {
        Joints target;
        if(module.read_target_rt(imp_->session,target,imp_->target_revision)) imp_->tracker.target(target);
    }
    imp_->tracker.step(actual,imp_->limits,stopping);
    for(std::size_t i=0;i<6;++i) motors[i].setTargetPos(imp_->tracker.position()[i]);
    const bool braking=stopping || imp_->tracker.faulted();
    module.publish_rt(braking,imp_->tracker.settled() && physical_stop && at_command,imp_->tracker.fault());
    // A stop acknowledgement never reports completion while braking/settling.
    if(braking && imp_->tracker.stopped() && physical_stop) return 0;
    return count();
}
void A10JointFollower::collectNrt() {
    if(imp_->module && executeRetCode()<0)
        imp_->module->publish_rt(true,false,"sdk_execution_error");
    if(imp_->module) imp_->module->finish(imp_->session);
    imp_->lease.reset();
}
KAANH_DEFINE_BIG_FOUR_CPP(A10JointFollower)
} // namespace a10_follower

ARIS_REGISTRATION {
    aris::core::class_<a10_follower::A10UdpFollower>("A10UdpFollower").inherit<kaanh::follower::UDPFollower>();
    aris::core::class_<a10_follower::A10JointFollower>("A10JointFollower").inherit<aris::plan::Plan>();
}
