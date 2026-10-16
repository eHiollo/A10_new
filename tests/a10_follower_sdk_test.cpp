#ifdef NDEBUG
#undef NDEBUG
#endif
#include "a10_follower.hpp"
#include "a10_follower_protocol.hpp"
#include "a10_motion_lease.hpp"
#include "kaanh/middleware/shell.hpp"
#include "kaanh/robot/robot.hpp"
#include "kaanhbot/authority/authority.hpp"
#include <cassert>
#include <chrono>
#include <iostream>

namespace {
class NoInitJointParam : public kaanh::follower::FollowerAjParam {
public: void init() override {} // SDK fixture: no trajectory managers or sockets.
};
class SimMotor : public aris::control::EthercatMotor {
public:
    double actual{}, commanded{}, velocity{}; bool enabled{true};
    double actualPos() const override { return actual; }
    double actualVel() const override { return velocity; }
    double targetPos() const override { return commanded; }
    void setTargetPos(double q) override { commanded=q; }
    bool isEnabled() const override { return enabled; }
    std::uint16_t statusWord() const override { return enabled ? 0x27 : 0x40; }
    void advance() { velocity=(commanded-actual)/a10_follower::Guard::dt;actual=commanded; }
};
}
int main() {
    using namespace a10_follower;
    const auto directory=std::filesystem::temp_directory_path()/ ("a10-follower-sdk-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(directory);set_config_directory(directory);
    auto& cs=aris::server::ControlServer::instance();
    cs.resetModel(new aris::dynamic::MultiModel);
    cs.resetCustomModule(new kaanh::robot::Robot);
    auto* shell=new kaanh::middleware::Shell;cs.resetMiddleWare(shell);
    auto* modules=new aris::core::PointerArray<kaanh::module::MiddleModule>;
    modules->add<kaanhbot::authority::Authority>().setName("Authority");
    auto& follower=modules->add<A10UdpFollower>();shell->setModulePool(modules);
    cs.planRoot().planPool().add<A10JointFollower>();
    auto& param=follower.followerParamPool().add<NoInitJointParam>();
    param.setSocketServer(new aris::core::SocketServer("test", "0", aris::core::SocketServer::Type::UDP));
    // Empty SDK parameter pool: register the original follower get callback,
    // without initializing motors, starting the controller or opening sockets.
    follower.init();
    auto state=kaanh::robot::Robot::instanceInCs().getJson(kaanh::robot::Robot::GetType::kInfo);
    std::cout<<"SDK follower status: "<<state.dump()<<std::endl;
    Joints lo,hi;lo.fill(-3);hi.fill(3);
    auto token=follower.begin("follower_aj",lo,hi);
    assert(!follower.isRunning());assert(follower.start_rt(token));
    assert(follower.isRunning());assert(follower.getFollowerMode()==kaanh::follower::FollowerMode::kAixs);
    std::string response;
    follower.execute("follower_config --max_vel=0.2 --max_acc=1 --max_dec=1 --save=1",[&](auto x){response=x;});
    assert(nlohmann::json::parse(response)["ret_code"]==0);
    assert(load_limits(directory/"follower_config.json",{}).max_vel==.2);
    aris::core::Msg packet("jp --motor_pos=[10,20,30,40,50,60]");follower.receive(packet.toString());
    Joints q;std::uint64_t revision=0;assert(follower.read_target_rt(token,q,revision));
    assert(std::abs(q[0]-3.141592653589793/18)<1e-12);
    follower.receive("jp --motor_pos=[0,0,0,0,0,999]");assert(!follower.read_target_rt(token,q,revision));
    follower.execute("stop_follower",[&](auto x){response=x;});
    assert(nlohmann::json::parse(response)==nlohmann::json({{"ret_code",0},{"ret_context",""},{"ret_msg",""}}));
    assert(follower.stop_requested() && follower.isRunning());
    follower.receive(packet.toString());assert(!follower.read_target_rt(token,q,revision));
    follower.finish(token);assert(!follower.isRunning());
    token=follower.begin("follower_aj",lo,hi);assert(follower.start_rt(token));
    revision=0;assert(!follower.read_target_rt(token,q,revision));
    follower.finish(token-1);assert(follower.isRunning());follower.finish(token);
    follower.execute("stop_follower",[&](auto x){response=x;});assert(nlohmann::json::parse(response)["ret_code"]==0);
    follower.execute("follower_config --max_vel=nan",[&](auto x){response=x;});assert(nlohmann::json::parse(response)["ret_code"]!=0);
    A10JointFollower plan;
    plan.parse("follower_aj");bool rejected=false;try{plan.prepareNrt();}catch(const std::exception&){rejected=true;}assert(rejected);
    // Run the actual Plan lifecycle against six explicitly simulated motors.
    // Neither cs.init(), cs.start(), enable(), nor real slave I/O is invoked.
    std::array<aris::control::EthercatSlave,6> slaves;
    cs.resetController(new aris::control::Controller);
    auto* motor_pool=&cs.controller().motorPool();
    for(std::size_t i=0;i<6;++i) {
        slaves[i].setVirtual(false);
        auto& motor=motor_pool->add<SimMotor>();motor.setSlave(&slaves[i]);
        motor.setMinPos(-3);motor.setMaxPos(3);
    }
    auto& robot=kaanh::robot::Robot::instanceInCs();
    robot.setRobotStatus(kaanh::robot::RobotStatus::kNormal);
    robot.setRobotActivate(kaanh::robot::RobotActivate::kEnabled);
    robot.setRobotMotion(kaanh::robot::RobotMotion::kStopped);
    plan.setControlServer(&cs);plan.setController(&cs.controller());plan.setModelBase(&cs.model());
    const auto reject_start = [&] {
        plan.parse("follower_aj"); bool denied=false;
        try { plan.prepareNrt(); } catch(const std::exception&) { denied=true; }
        assert(denied && motion_owner.load()==MotionOwner::None);
    };
    robot.setRobotActivate(kaanh::robot::RobotActivate::kDisabled);reject_start();
    robot.setRobotActivate(kaanh::robot::RobotActivate::kEnabled);
    robot.setRobotStatus(kaanh::robot::RobotStatus::kError);reject_start();
    robot.setRobotStatus(kaanh::robot::RobotStatus::kNormal);
    robot.setRobotMotion(kaanh::robot::RobotMotion::kRunning);reject_start();
    robot.setRobotMotion(kaanh::robot::RobotMotion::kStopped);
    auto& first_motor=dynamic_cast<SimMotor&>((*motor_pool)[0]);
    first_motor.enabled=false;reject_start();first_motor.enabled=true;
    first_motor.actual=std::numeric_limits<double>::quiet_NaN();reject_start();first_motor.actual=0;
    plan.parse("follower_aj");plan.prepareNrt();assert(!follower.isRunning());
    assert(follower.diagnostics()["max_vel"]==.2);
    plan.setCount(1);assert(plan.executeRT()>0 && follower.isRunning());
    follower.receive(packet.toString());
    for(int tick=2;tick<4000;++tick) {
        plan.setCount(tick);assert(plan.executeRT()>0);
        for(auto& motor:*motor_pool) dynamic_cast<SimMotor&>(motor).advance();
    }
    assert(follower.diagnostics()["settled"]==true);
    state=robot.getJson(kaanh::robot::Robot::GetType::kInfo);
    assert(state.at("follower_msg").at("running_state")==true);
    assert(state.at("follower_msg").at("follower_mode")=="Axis");
    follower.execute("stop_follower",[&](auto x){response=x;});assert(follower.isRunning());
    bool stopped=false;
    for(int tick=4000;tick<5000;++tick) {
        plan.setCount(tick);const auto remaining=plan.executeRT();
        for(auto& motor:*motor_pool) dynamic_cast<SimMotor&>(motor).advance();
        if(remaining==0){stopped=true;break;}
    }
    assert(stopped);plan.collectNrt();assert(!follower.isRunning());
    state=robot.getJson(kaanh::robot::Robot::GetType::kInfo);assert(state.at("follower_msg").at("running_state")==false);
    // Stop while moving: immediate acknowledgement must keep running_state
    // true until bounded command braking and simulated feedback both stop.
    plan.parse("follower_aj");plan.prepareNrt();plan.setCount(1);assert(plan.executeRT()>0);
    follower.receive("jp --motor_pos=[-20,-20,-20,-20,-20,-20]");
    for(int tick=2;tick<180;++tick) {
        plan.setCount(tick);assert(plan.executeRT()>0);
        for(auto& motor:*motor_pool) dynamic_cast<SimMotor&>(motor).advance();
    }
    follower.execute("follower_config --max_vel=0.1 --max_acc=0.8 --max_dec=0.8",[&](auto x){response=x;});
    assert(nlohmann::json::parse(response)["ret_code"]==0);
    follower.execute("stop_follower",[&](auto x){response=x;});assert(follower.isRunning());
    stopped=false;
    for(int tick=180;tick<1500;++tick) {
        plan.setCount(tick);const auto remaining=plan.executeRT();
        for(auto& motor:*motor_pool) dynamic_cast<SimMotor&>(motor).advance();
        if(remaining==0){stopped=true;break;}
    }
    assert(stopped);plan.collectNrt();assert(!follower.isRunning());
    // Cancellation before the first RT tick never arms or moves the follower.
    const double held=(*motor_pool)[0].targetPos();
    plan.parse("follower_aj");plan.prepareNrt();
    follower.execute("stop_follower",[&](auto x){response=x;});
    follower.receive(packet.toString());plan.setCount(1);assert(plan.executeRT()==0);
    assert((*motor_pool)[0].targetPos()==held);plan.collectNrt();assert(!follower.isRunning());
    // The admission check rejects virtual feedback even when enabled flags agree.
    slaves[0].setVirtual(true);plan.parse("follower_aj");rejected=false;
    try{plan.prepareNrt();}catch(const std::exception&){rejected=true;}assert(rejected);
    slaves[0].setVirtual(false);
    follower.execute("follower_aj",[&](auto x){response=x;});
    assert(nlohmann::json::parse(response)["ret_code"]!=0); // controller not started
    // Inherited reflection must preserve the SDK pool used by deployed XML.
    A10UdpFollower restored;
    aris::core::fromXmlString(restored,"<A10UdpFollower name=\"UDPFollower\"><FollowerParamPool __prop_name__=\"follower_param_pool\"><FollowerAjParam tg_id=\"0\"><SocketServer __prop_name__=\"socket_server\" connect_type=\"UDP\" port=\"9999\"/></FollowerAjParam></FollowerParamPool></A10UdpFollower>");
    assert(restored.followerParamPool().size()==1);
    assert(restored.followerParamPool()[0].socketServer().port()=="9999");
    std::filesystem::remove_all(directory);
    std::cout<<"follower SDK lifecycle/commands/serialization tests passed\n";
}
