#include "template.hpp"
#include "kaanh/log/log.hpp"

namespace kaanhtemplate{
    //日志报错:需提前在模块中注册错误，请勿在plan中注册，重复注册同一个错误码会抛异常
    //错误码分配：
    //yw:0x1000 ~ 0x1fff
    //xxt:0x2000 ~ 0x2fff
    //lcz:0x3000 ~ 0x3fff
    //kaanh::log::Sqlite3Log::instance().registerError(int 错误码,const std::string& 错误信息,const std::string & 错误处理措施);//前两个参数必传，第三个参数错误处理措施可后续通过addMeassure函数添加或修改
    //kaanh::log::Sqlite3Log::instance().addMeassure(int 错误码,const std::string& 错误处理措施)//添加或修改错误码的对应错误处理措施
    //cpp中的plan模板
    struct PlanName::Imp
    {
        //...
    };
    auto PlanName::prepareNrt() -> void
    {
        
        //...
        //setPlanPreError(int 错误码,*this,const std::string& 错误信息前缀,const std::string& 错误信息后缀,kaanh::log::LogType::错误类型);//前两个参数必传
        kaanh::log::Sqlite3Log::instance().setPlanPreError(0x2000,*this,LOCALE_SELECT("","错误信息前缀"),LOCALE_SELECT("","错误信息后缀"),kaanh::log::LogType::kSystem);
        //option() |= NOT_RUN_EXECUTE_FUNCTION；//报错后自己设置是否执行后续execute等！！！并返回!
        return -0x2000;
    }
    auto PlanName::executeRT() -> int
    {
        //...
        //setPlanExeError(错误码,*this,错误信息前缀，错误信息后缀，kaanh::log::LogType::错误类型);//前两个参数必传
        kaanh::log::Sqlite3Log::instance().setPlanExeError(0x2001,*this,LOCALE_SELECT("","错误信息前缀"),LOCALE_SELECT("","错误信息后缀"),kaanh::log::LogType::kSystem);
        //option() |= ...//报错后自己设置是否执行后续collect等！！！并返回!
        return -0x2001;
    }
    PlanName::PlanName(const std::string &name) : imp_(new Imp)
    {
        aris::core::fromXmlString(command(),
                                  "<Command name=\"planname\">"
                                  "	<GroupParam>"
                                  "		<Param name=\"pos\" default=\"JointTarget{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
                                  "		<Param name=\"reinit\" default=\"0\" />"
                                  "	</GroupParam>"
                                  "</Command>");
    }
    PlanName::~PlanName() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PlanName)

    //cpp中的module模板
    struct ModuleName::Imp {
        //...
    };
    
    ModuleName::ModuleName() : imp_(new Imp) {}
    ModuleName::~ModuleName() = default;
    auto ModuleName::function1()->void{
        //..
    }
    auto ModuleName::init()->void {
        //...
        //registerError(int 错误码,const std::string& 错误信息,const std::string & 错误处理措施);//前两个参数必传，第三个参数错误处理措施可后续通过addMeassure函数添加或修改
        kaanh::log::Sqlite3Log::instance().registerError(0x2002,LOCALE_SELECT("","错误信息"),LOCALE_SELECT("","错误处理措施"));
        //setError(错误码,错误信息前缀,错误信息后缀,kaanh::log::LogType::错误类型);//第一个参数必传
        kaanh::log::Sqlite3Log::instance().setError(0x2002,LOCALE_SELECT("","错误信息前缀"),LOCALE_SELECT("","错误信息后缀"),kaanh::log::LogType::kSystem);
        //registerState(int 状态码,const std::string& 状态名);//注册状态，状态码和错误码共用一个数组，请勿重复，最好通过addmeassure函数添加和修改处理措施！
        kaanh::log::Sqlite3Log::instance().registerState(0x2003, LOCALE_SELECT("","状态名"));
        kaanh::log::Sqlite3Log::instance().addMeassure(0x2003, LOCALE_SELECT("","状态处理措施"));
        //triggerState(int 状态码);//通过此方式触发状态则必须在代码中通过disappearState(int 状态码)来消除状态
        kaanh::log::Sqlite3Log::instance().triggerState(0x2003);
        //也可以用setError函数报错此状态码，此时的状态报错可以清除
        kaanh::log::Sqlite3Log::instance().setError(0x2002,LOCALE_SELECT("","状态被触发"),LOCALE_SELECT("","错误信息后缀"),kaanh::log::LogType::kSystem);
    }
    
    auto ModuleName::execute(const std::string &str, std::function<void(std::string)> send_ret) noexcept->std::pair<std::string, std::string> {
        //...
        //execute报错同init中一样
        END_CMD_FLOW;
    }
    auto ModuleName::instanceInCs()->ModuleName& {
        return *dynamic_cast<ModuleName*>(&(kaanh::middleware::Shell::instanceInCs().getModule("ModuleName")));
    }

    ARIS_REGISTRATION
    {
    aris::core::class_<PlanName>("PlanName").inherit<aris::plan::Plan>();

    aris::core::class_<ModuleName>("ModuleName")
    .inherit<kaanh::module::MiddleModule>()
    ;

    }
}