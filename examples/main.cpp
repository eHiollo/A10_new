#include "aris.hpp"
#include "kaanh/log/log.hpp"
#include "kaanh/robot/robot.hpp"
#include "kaanhbot/robot/robot.hpp"
#include "kaanhbot/config/kaanhconfig.hpp"

auto xmlpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
auto logpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string logfolder = "log";

int main(int argc, char *argv[]){
	xmlpath = xmlpath / xmlfile;
	logpath = logpath / logfolder;

	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);
	auto path = argc < 2 ? xmlpath : argv[2];
	auto logp = argc < 2 ? logpath : argv[3];

	std::cout << "port:" << port << std::endl;
	std::cout << "xmlpath:" << xmlpath << std::endl;
	std::cout << "path:" << path << std::endl;
	std::cout << "logfolder:" << logp << std::endl;

    try {
      aris::core::fromXmlFile(cs, path);
    }
    catch (std::runtime_error&e) {
    std::cout << e.what() << std::endl;
    }
    cs.init();
    auto &model = dynamic_cast<aris::dynamic::MultiModel&>(cs.model());

    double output0[6]{ -157.354000 / 1000,-424.003000 / 1000,430.562000 / 1000,112.436000 / 180 * aris::PI,7.388000 / 180 * aris::PI,181.057000 / 180 * aris::PI };
    double output[6]{ -153.059000 / 1000,425.414000 / 1000,430.019000 / 1000,332.579000 / 180 * aris::PI,7.320000 / 180 * aris::PI,180.993000 / 180 * aris::PI };
    aris::dynamic::Marker *tool = model.tools()[0];
    aris::dynamic::Marker *wobj = model.wobjs()[0];
    tool->setPe(*wobj, output0, "321");
    model.updP();
    model.inverseKinematics();
    double input0[6];
    cs.model().getInputPos(input0);
    aris::dynamic::dsp(1, 6, input0);

    tool->setPe(*wobj, output, "321");
    model.updP();
    model.inverseKinematics();
    double input[6];
    cs.model().getInputPos(input);
    aris::dynamic::dsp(1, 6, input);

	aris::core::fromXmlFile(cs, path);
    //cs.resetModel(kaanhbot::config::createModelLansi().release());
	cs.init();
    //aris::core::toXmlFile(cs, path);

    std::cout <<"motor size: "<< cs.controller().motorPool().size() << std::endl;
    std::cout <<"io size: "<< cs.controller().digitalIoPool().size() << std::endl;
    std::cout <<"fs size: "<< cs.controller().ftSensorPool().size() << std::endl;

	// 防止kaanhbot库被编译器优化
	kaanhbot::robot::Robot::instanceInCs();

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

	// 重载Aris Rt-log接口
    auto func = [](aris::plan::Plan *p, int error_num, const char *error_msg) {
        RT_ERROR_SYS_NC(error_msg, error_num);
    };
    cs.setRtErrorCallback(func);

    //实时回调函数，每个实时周期调用一次
    cs.setRtPlanPreCallback([](aris::server::ControlServer&cs){
        static int32_t update_counter = 0;
        if(update_counter < 2000){
            update_counter++;
            return;
        }
        kaanh::robot::Robot::instanceInCs().rtUpdate(cs);
    });

    //开启控制器服务
    try {
        cs.start();
		INFO_SYS(LOCALE_SELECT("system starts", "系统启动"), 0)
    }
    catch (std::exception& err){
        ERROR_SYS_NC(LOCALE_SELECT("failed to start system, please reboot",
								   "系统启动失败, 请重启控制器"), -1);
    }


    //电机抱闸设置恢复
    int driver_num=cs.controller().motorPool().size();
    for(int i= 0; i < driver_num; i++){
        auto &motor=cs.controller().motorPool().at(i);
        motor.setOutputIoNrt(1,0x00);
        motor.setOutputIoNrt(2,0x00);
    }

    cs.executeCmd("md");

    cs.executeCmd("rc");

	//Start Web Socket
	cs.open();

	//Receive Command
	cs.runCmdLine();

	return 0;
}
