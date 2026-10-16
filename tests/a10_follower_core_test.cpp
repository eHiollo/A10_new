#ifdef NDEBUG
#undef NDEBUG
#endif
#include "a10_follower_protocol.hpp"
#include "a10_motion_lease.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace a10_follower;
namespace {
Joints filled(double x) { Joints q; q.fill(x); return q; }
template<class F> void rejects(F f) { bool failed=false; try { f(); } catch (const std::exception&) { failed=true; } assert(failed); }
Tracker tracker() { Tracker t; assert(t.reset({}, {}, filled(-3), filled(3))); return t; }
void track(Tracker& t, const Limits& l, bool stop=false) {
    const auto p=t.position(), v=t.velocity(); t.step(p,l,stop);
    for(std::size_t i=0;i<6;++i) {
        assert(std::isfinite(t.position()[i]));
        const double next=t.velocity()[i];
        const double rate=(v[i]*next>=0 && std::abs(next)>std::abs(v[i]))?l.max_acc:l.max_dec;
        assert(std::abs(next-v[i]) <= rate*Guard::dt+1e-9);
        // Lowered working velocity may be exceeded only while decelerating.
        assert(std::abs(next)<=std::max(std::abs(v[i]),std::min(l.max_vel,Guard::speed_limit(i)))+1e-9);
        assert(std::abs(t.position()[i]-p[i]-next*Guard::dt)<1e-12);
    }
    assert(!t.faulted());
}
}
int main() {
    auto q=parse_jp("jp --motor_pos=[-117.13,-46.66,136.69,-32.32,-70.14,-13.99]\0");
    assert(std::abs(q[0]+117.13*3.141592653589793/180)<1e-12);
    for(const auto* body:{"jp --motor_pos=[1,2]","jp --motor_pos=[1,2,3,4,5,true]","jp --motor_pos=[1,2,3,4,5,null]","jp --motor_pos=[1,2,3,4,5,6,7]","jp --motor_pos=[1,2,3,4,5,6] xx","bad --motor_pos=[1,2,3,4,5,6]"}) rejects([&]{parse_jp(body);});
    rejects([]{parse_config("follower_config --max_vel=nan",{});});
    rejects([]{parse_config("follower_config --max_acc=0",{});});
    rejects([]{parse_config("follower_config --max_vel=1",{});});
    rejects([]{parse_config("follower_config --max_jerk=1",{});});
    rejects([]{parse_config("follower_config --max_vel=0.1 --max_vel=0.2",{});});
    rejects([]{parse_config("follower_aj --save=1",{},false);});
    assert(parse_config("follower_config --max_dec=1 --save=1",{}).save);
    auto a=tracker(), b=tracker(); Limits limits;
    assert(a.target(filled(.5))); assert(b.target(filled(.5)));
    for(int tick=0;tick<4000;++tick) {
        // Repeat the same target at 500 Hz vs only once: identical commands.
        assert(b.target(filled(.5))); track(a,limits); track(b,limits);
        assert(a.position()==b.position());
    }
    assert(a.settled());
    assert(!a.target(filled(3))); assert(!a.target(filled(INFINITY)));
    auto t=tracker(); t.target(filled(1));
    for(int i=0;i<300;++i) track(t,limits);
    limits={.05,.4,.5};
    for(int i=0;i<1000;++i) track(t,limits);
    assert(std::abs(t.velocity()[0])<=.05+1e-8);
    t.target(filled(-.1));
    for(int i=0;i<12000;++i) track(t,limits);
    assert(t.settled());
    // Bursts replace the destination; no historical path is replayed.
    t=tracker(); for(int i=0;i<1000;++i) t.target(filled(i==999 ? -.2 : .5));
    for(int i=0;i<3000;++i) track(t,{});
    assert(t.settled() && std::abs(t.position()[0]+.2)<1e-6);
    t=tracker();t.target(filled(1));for(int i=0;i<300;++i)track(t,{});
    const auto before=t.position();for(int i=0;i<1000;++i)track(t,{},true);
    assert(t.stopped() && t.position()[0]>=before[0] && t.position()[0]<1);
    // Feedback faults latch and brake; new targets never clear the fault.
    t=tracker();t.target(filled(1));for(int i=0;i<100;++i)track(t,{});
    t.step(filled(.5),{});assert(t.faulted());t.target({});assert(t.faulted());
    rejects([]{auto first=MotionLease::acquire(MotionOwner::Follower);auto second=MotionLease::acquire(MotionOwner::Policy);});
    assert(motion_owner.load()==MotionOwner::None);
    Mailbox<6> mailbox;std::atomic<bool> done{false};
    std::thread writer([&]{for(std::uint64_t i=1;i<100000;++i)mailbox.publish(filled(double(i)),i,i);done=true;});
    do { Mailbox<6>::Snapshot value;if(mailbox.read(value)){for(double x:value.values)assert(x==double(value.token));assert(value.stamp==value.token);} }while(!done.load());writer.join();
    const auto directory=std::filesystem::temp_directory_path()/ ("a10-follower-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(directory);const auto path=directory/"follower_config.json";
    assert(load_limits(path,{}).max_vel==.3);save_limits(path,{.2,1,1.2});assert(load_limits(path,{}).max_dec==1.2);
    rejects([&]{save_limits(directory/"missing"/"config.json",{});});
    assert(load_limits(path,{}).max_vel==.2);
    std::ofstream(path)<<"broken";rejects([&]{load_limits(path,{});});
    std::filesystem::remove_all(directory);
    std::cout<<"follower core/protocol/config/concurrency tests passed\n";
}
