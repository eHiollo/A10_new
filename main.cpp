#include "aris.hpp"
#include "kaanh/log/log.hpp"
#include "kaanh/robot/robot.hpp"
#include "kaanhbot/robot/robot.hpp"
#include "kaanhbot/config/kaanhconfig.hpp"
#include "aris_version.h"
#include "kaanh_version.h"
#include "kaanhbot_version.h"
#include "kaanhbin_version.h"
#include "aris/core/version.hpp"
#include "kaanh/general/version.hpp"
#include "kaanhbot/system/version.hpp"
#include "kaanhbot/system/about.hpp"
#include "kaanhbot/utility/tool_api.hpp"
#include "plan.hpp"
#include "a10_gripper_bridge.hpp"
#include "a10_tcp_server.hpp"
#include "a10_vr_plan.hpp"
#include "gripper.hpp"

#include "robot.hpp"
#include "assemcomd.hpp"
#include "kaanh/middleware/shell.hpp"
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>
#define __S(x) #x
#define _S(x) __S(x)

#ifndef KAANH_SOURCE_DIR
#define KAANH_SOURCE_DIR ""
#endif

//建立全局变量 让其他地方可以调用
A10TcpServer* g_tcp_server = nullptr;
BusServo* g_gripper = nullptr;

auto logpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string logfolder = "log";

auto resolve_kaanh_xml() -> std::filesystem::path
{
    std::error_code ec;
    std::vector<std::filesystem::path> candidates;

    if constexpr (KAANH_SOURCE_DIR[0] != '\0')
    {
        candidates.emplace_back(std::filesystem::path(KAANH_SOURCE_DIR) / xmlfile);
    }

    const auto cwd = std::filesystem::current_path(ec);
    if (!ec)
    {
        for (auto p = cwd; ; p = p.parent_path())
        {
            candidates.emplace_back(p / xmlfile);
            if (p == p.parent_path())
            {
                break;
            }
        }
    }

    for (const auto& c : candidates)
    {
        std::error_code fec;
        if (std::filesystem::exists(c, fec) && !fec)
        {
            const auto sz = std::filesystem::file_size(c, fec);
            if (!fec && sz > 0)
            {
                return std::filesystem::absolute(c);
            }
        }
    }
    throw std::runtime_error("kaanh.xml not found (expected repo root kaanh.xml)");
}

int main(int argc, char *argv[]){
	logpath = logpath / logfolder;

	auto& cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);
	auto path = argc >= 3 ? std::filesystem::path(argv[2]) : resolve_kaanh_xml();
	auto logp = argc >= 4 ? argv[3] : logpath;

	std::cout << "port:" << port << std::endl;
	std::cout << "xmlpath:" << path << std::endl;
	std::cout << "logfolder:" << logp << std::endl;

	// 重载Aris log接口
	aris::core::setLogMethod([](aris::core::LogData data)->void {
		// switch (data.level) {
		// case aris::core::LogLvl::kError :
		// case aris::core::LogLvl::kFatal :
		// 	ERROR_SYS_NC("Aris -- "+data.msg, data.code);
		// 	break;
		// case aris::core::LogLvl::kDebug :
		// case aris::core::LogLvl::kInfo :
		// default :
		// 	break;
		// } 
		if ((int)(data.level)>(int)(aris::core::LogLvl::kInfo))
			std::cout << "aris-log -- " << data.msg << std::endl;
	});

	aris::core::fromXmlFile(cs, path);

    try
    {
        auto& shell = kaanh::middleware::Shell::instanceInCs();
        auto& pool = shell.modulePool();
        bool has = false;
        for (aris::Size i = 0; i < pool.size(); ++i)
        {
            if (pool[i].name() == "VrReset")
            {
                has = true;
                break;
            }
        }
        if (!has)
        {
            pool.add<a10_tcp::A10VrResetModule>();
            std::cout << "VrReset: intercept reset in Shell (not Plan queue)" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "VrReset install failed: " << e.what() << std::endl;
    }
    
    //建立TCP服务，与lerobot通信
    static A10TcpServer tcp_server;
    if(tcp_server.start(8080)){ 
        std::cout <<"TCP Server started at port 8080" << std::endl;
        g_tcp_server = &tcp_server;
    } else {
        std::cerr <<"Faild to start tcp server " << std::endl;
    }
    
    static std::unique_ptr<BusServo> gripper_holder;
    try
    {
        gripper_holder = std::make_unique<BusServo>("/dev/ttyUSB0", 1000000, 150, false);
        if (gripper_holder->ping(10) == 0)
        {
            g_gripper = gripper_holder.get();
            std::cout << "Gripper: connected (USB)" << std::endl;
        }
        else
        {
            gripper_holder.reset();
            std::cout << "Gripper: not detected, arm-only mode" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        gripper_holder.reset();
        g_gripper = nullptr;
        std::cout << "Gripper: disabled (" << e.what() << "), arm-only mode" << std::endl;
    }
    std::thread([]() { a10_tcp::run_gripper_service_loop(g_gripper); }).detach();

    cs.init();

    // 独立线程：仅把**当前关节反馈**写入 TCP 侧 robot_q_，供外部 GET_FOLLOWER_STATE / GET_LEADER_STATE 查询。
    // 必须在 cs.init() 之后启动，避免电机池尚未建好时读 actualPos。
    std::thread state_update_thread([&]() {
        while (true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));

            // 协议仍为 13 维：前 6 为单臂关节，6..11 在单臂上填 0，[12] 为夹爪。
            std::vector<double> current_q(13, 0.0);
            auto& motors = cs.controller().motorPool();
            const aris::Size n_motors = motors.size();
            const aris::Size n_arm = n_motors < 6 ? n_motors : static_cast<aris::Size>(6);
            for (aris::Size i = 0; i < n_arm; ++i)
            {
                current_q[i] = motors[i].actualPos();
            }
            current_q[12] = a10_tcp::g_vr_grip_actual_mm.load(std::memory_order_acquire);

            // chunk 执行期间不刷新 ``robot_q_``，客户端应轮询 ``GET_POLICY_STATUS`` 至 idle 后再 ``GET_FOLLOWER_STATE`` 做观测/推理。
            if (g_tcp_server != nullptr && !g_tcp_server->policy_batch_queue_empty())
            {
                continue;
            }

            tcp_server.send_set_joints(current_q);
        }
    });
    state_update_thread.detach();

    // 修改末端杆件位姿,需要重写xml
   {
       auto& multimodel = dynamic_cast<aris::dynamic::MultiModel&>(cs.model());
       if (multimodel.subModels().empty())
       {
           std::cerr << "startup: no submodel in MultiModel" << std::endl;
       }
       else
       {
       auto& model = dynamic_cast<aris::dynamic::Model&>(multimodel.subModels()[0]);
        double input_pos[6]{0,0,0,0,0,0};
        model.setInputPos(input_pos);
        model.forwardKinematics();

       if (model.name() == "PumaModel")
       {
            auto* ee = model.findPart("EE");
            if (ee && ee->geometryPool().size())
            {
            auto pm = *ee->pm();
            auto& geo = dynamic_cast<aris::dynamic::FileGeometry&>(ee->geometryPool()[0]);
            std::string path = geo.filePath();
            ee->geometryPool().clear();
            ee->geometryPool().add<aris::dynamic::FileGeometry>(path,pm);
            }
       }
        else if (model.name() == "UrModel")
        {
            auto* l6 = model.findPart("L6");
            if (l6 && l6->geometryPool().size())
            {
            auto pm = *l6->pm();
            auto& geo = dynamic_cast<aris::dynamic::FileGeometry&>(l6->geometryPool()[0]);
            std::string path = geo.filePath();
            l6->geometryPool().clear();
            l6->geometryPool().add<aris::dynamic::FileGeometry>(path,pm);
            }
        }
        else if (model.name() == "ScaraModel")
        {
            auto* l4 = model.findPart("L4");
            if (l4 && l4->geometryPool().size())
            {
            auto pm = *l4->pm();
            auto& geo = dynamic_cast<aris::dynamic::FileGeometry&>(l4->geometryPool()[0]);
            std::string path = geo.filePath();
            l4->geometryPool().clear();
            l4->geometryPool().add<aris::dynamic::FileGeometry>(path,pm);
            }
        }
        else if (model.name() == "DeltaModel")
        {

        }
        else{}
        try {
            kaanhbot::utility::saveCs();
        } catch (const std::exception& e) {
            std::cerr << "startup: saveCs skipped (" << e.what() << ")" << std::endl;
        }
       }
   }
	// 重载Aris log接口
	aris::core::setLogMethod([](aris::core::LogData data)->void {
		// switch (data.level) {
		// case aris::core::LogLvl::kError :
		// case aris::core::LogLvl::kFatal :
		// 	ERROR_SYS_NC("Aris -- "+data.msg, data.code);
		// 	break;
		// case aris::core::LogLvl::kDebug :
		// case aris::core::LogLvl::kInfo :
		// default :
		// 	break;
		// } 
		std::cout << "aris-log -- " << data.msg << std::endl;
	});

    std::cout <<"motor size: "<< cs.controller().motorPool().size() << std::endl;
	std::cout <<"model motion size: "<< cs.model().inputPosSize() << std::endl;
    std::cout <<"io size: "<< cs.controller().digitalIoPool().size() << std::endl;
    std::cout <<"fs size: "<< cs.controller().ftSensorPool().size() << std::endl;

	// 设置版本回调
    kaanhbot::system::About::registerCbk("aris", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(ARIS_VERSION_MAJOR) + "." + _S(ARIS_VERSION_MINOR) + "." + _S(ARIS_VERSION_PATCH) + "." + _S(ARIS_VERSION_TIME)), std::string(_S(ARIS_DESCRIPTION)));
        }, aris::core::version);
    kaanhbot::system::About::registerCbk("kaanh", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(KAANH_VERSION_MAJOR) + "." + _S(KAANH_VERSION_MINOR) + "." + _S(KAANH_VERSION_PATCH) + "." + _S(KAANH_VERSION_TIME)), std::string(_S(KAANH_DESCRIPTION)));
        }, kaanh::general::version);
    kaanhbot::system::About::registerCbk("kaanhbot", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(KAANHBOT_VERSION_MAJOR) + "." + _S(KAANHBOT_VERSION_MINOR) + "." + _S(KAANHBOT_VERSION_PATCH) + "." + _S(KAANHBOT_VERSION_TIME)), std::string(_S(KAANHBOT_DESCRIPTION)));
        }, kaanhbot::system::version);
    kaanhbot::system::About::registerCbk("kaanhbin", []()->std::pair<std::string, std::string> {
        return std::make_pair(std::string(std::string("v") + _S(KAANHBIN_VERSION_MAJOR) + "." + _S(KAANHBIN_VERSION_MINOR) + "." + _S(KAANHBIN_VERSION_PATCH) + "." + _S(KAANHBIN_VERSION_TIME)), std::string(_S(KAANHBIN_DESCRIPTION)));
        }, []()->std::pair<std::string, std::string> {
            return std::make_pair(std::string(std::string("v") + _S(KAANHBIN_VERSION_MAJOR) + "." + _S(KAANHBIN_VERSION_MINOR) + "." + _S(KAANHBIN_VERSION_PATCH) + "." + _S(KAANHBIN_VERSION_TIME)), std::string(_S(KAANHBIN_DESCRIPTION)));
        });

	// 防止kaanhbot库被编译器优化
	kaanhbot::robot::Robot::instanceInCs();

	// 重载Aris Rt-log接口
    auto func = [](aris::plan::Plan *p, int error_num, const char *error_msg) {
        auto err_num_abs = std::abs(error_num);
        if((err_num_abs & 0xffff) < 0x1000) RT_ERROR_SYS_NC(error_msg, error_num)

    };
    cs.setRtErrorCallback(func);

    try{
        auto & log = kaanh::log::Sqlite3Log::instance();
        log.initErrorConfig();
        log.createErrorConfig();
    }catch(const std::exception& err){
        WARNING_CFG_NC(LOCALE_SELECT("not find error.txt","未发现错误配置文件"),-1);
    }
    //实时回调函数，每个实时周期调用一次
    cs.setRtPlanPreCallback([](aris::server::ControlServer& cs) {
        static int32_t update_counter = 0;
        if (update_counter < 2000)
        {
            update_counter++;
            return;
        }
        kaanh::robot::Robot::instanceInCs().rtUpdate(cs);
    });

    // for(aris::Size i = 0; i < cs.controller().motorPool().size(); i++){
    //     cs.idleMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    //     cs.globalMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
    // }

    
    //开启控制器服务
    try {
        cs.start();
        INFO_SYS(LOCALE_SELECT("system starts", "系统启动"), 0)
        #ifdef __aarch64__
        int counts = 60;
        std::string filename="doc/tmp_slave.txt";
        if(!std::filesystem::exists(filename)){
            if((!std::filesystem::is_directory("./doc") )) std::filesystem::create_directory("doc");
            std::fstream fstrm(filename, std::ios::out | std::ios::trunc);
            fstrm.close();
        }
        while(counts-->0){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if(system(("ethercat slave > " + filename).c_str())!=0) continue;
            std::ifstream  ifs(filename);
            std::string ethercat_slave_str;
            int i=0;
            bool flag{true};
            while(getline(ifs,ethercat_slave_str)&&i<100){
                // 0  0:0  PREOP + CoolDrive RC
                // 0  0:0  PREOP E CoolDriver
                // 0  0:0  SAFEOP+ERROR E CoolDrive  RC
                std::regex ruler("[0-9]+\\s+[0-9]+:[0-9]+\\s+([[:w:](+)(0-9)(_)]+)\\s+[[:w:](+)(0-9)(_)]+\\s+(.+)");
                std::sregex_iterator pos(ethercat_slave_str.cbegin(),ethercat_slave_str.cend(),ruler);
                std::sregex_iterator end;
                std::string ethercat_status;
		        // std::string ethercat_name;
                if(pos!=end){
                    ethercat_status=pos->str(1);
		            // ethercat_name=pos->str(2);
			        // std::cout << ethercat_status << ":" << ethercat_name << std::endl;
                    if(ethercat_status != "OP"){
                        flag = false;
                    }
                }
                i++;
            }
            if(flag) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        #endif
        //电机抱闸设置恢复
//        int driver_num=cs.controller().motorPool().size();
//        for(int i= 0; i < driver_num; i++){
//            auto &motor=cs.controller().motorPool().at(i);
//            motor.setOutputIoNrt(1,0x00);
//            motor.setOutputIoNrt(2,0x00);
//        }

        cs.executeCmd("md");

        cs.executeCmd("rc");
    }
    catch (const std::exception& err){
        kaanh::log::Sqlite3Log::instance().triggerState(0x3509);
    }

    
	//Start Web Socket
	cs.open();

	//Receive Command
	cs.runCmdLine();

	return 0;
}
