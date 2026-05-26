#include "user_slave/user_slave.hpp"

#include "kaanh/middleware/shell.hpp"
#include "kaanh/general/md5.hpp"
#include "kaanh/robot/robot.hpp"
#include "aris/server/server.hpp"
#include "aris/core/core.hpp"

namespace kaanhbot::user {

using kaanh::module::MiddleModule;
using aris::server::ControlServer;
using aris::server::Interface;
using aris::core::Command;
using aris::core::CommandParser;


	auto Uint32ToHexStr(void* data)->std::string {
		auto num = *reinterpret_cast<std::uint32_t*>(data);
		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::uint32_t) * 2) << std::hex << static_cast<std::uint32_t>(num);
		return s.str();
	}
	auto Uint16ToHexStr(void* data)->std::string {
		auto num = *reinterpret_cast<std::uint16_t*>(data);
		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::uint16_t) * 2) << std::hex << static_cast<std::uint32_t>(num);
		return s.str();
	}
	auto Uint8ToHexStr(void* data)->std::string {
		auto num = *reinterpret_cast<std::uint8_t*>(data);
		std::stringstream s;
		s << "0x" << std::setfill('0') << std::setw(sizeof(std::uint8_t) * 2) << std::hex << static_cast<std::uint32_t>(num);
		return s.str();
	}
auto PdoEntryInfo::setIndex(std::uint16_t index)->void { index_ = index; }
auto PdoEntryInfo::index()const->std::uint16_t { return index_; }
auto PdoEntryInfo::setSubindex(std::uint8_t subindex)->void { subindex_ = subindex; }
auto PdoEntryInfo::subindex()const->std::uint8_t { return subindex_; }
auto PdoEntryInfo::setBitSize(aris::Size size)->void { bitsize_ = size; }
auto PdoEntryInfo::bitSize()const->aris::Size { return bitsize_; }

auto GetSlaveParam::pdoInfoPool()->std::vector<PdoEntryInfo>& { return pdo_info_;}
auto GetSlaveParam::setPdoInfoPool(const std::vector<PdoEntryInfo> &info)->void { pdo_info_ = info;}
GetSlaveParam::GetSlaveParam() =default;//: imp_(new Imp) {}
GetSlaveParam::~GetSlaveParam() = default;

struct UserSlave::Imp {
    aris::core::CommandParser parser_;
	std::unique_ptr<aris::core::PointerArray<GetSlaveParam>> get_params_{new aris::core::PointerArray<GetSlaveParam>};

	std::vector<int> slave_ids{0};
};
auto UserSlave::paramPool()->aris::core::PointerArray<GetSlaveParam>& { return *imp_->get_params_; }
auto UserSlave::resetParamPool(aris::core::PointerArray<GetSlaveParam> *pool)->void { imp_->get_params_.reset(pool); }
auto UserSlave::setSlaveIds(const std::vector<int> slave_ids)->void { imp_->slave_ids = slave_ids;}
auto UserSlave::slaveIds()->const std::vector<int> { return imp_->slave_ids;}
UserSlave::UserSlave() : imp_(new Imp) {
}
UserSlave::~UserSlave() = default;
auto UserSlave::init()->void {

    imp_->parser_.init();

    kaanh::robot::Robot::instanceInCs().registerGetFunc([this](aris::server::ControlServer&cs)->nlohmann::json{
        nlohmann::json ret;

        auto func_get_slave = [&](GetSlaveParam& slave_param)->nlohmann::json 
		{
			nlohmann::json ret;
						
			//获取实时数据//
			std::any param = slave_param;
			if (cs.running()) 
			{
				auto& slave = dynamic_cast<aris::control::EthercatSlave&>(cs.master().slavePool()[slave_param.slave_id_]);
				cs.getRtData([&](aris::server::ControlServer& cs, const aris::plan::Plan *target, std::any& data)->void
				{
					auto &get_param = std::any_cast<GetSlaveParam&>(data);
					for (auto& info : get_param.pdoInfoPool())
					{
						if (info.is_uint_)
						{
							if (info.bitSize() == 8)
								slave.readPdo(info.index(),info.subindex(),&info.uint8,info.bitSize());
							else if (info.bitSize() == 16)
								slave.readPdo(info.index(),info.subindex(),&info.uint16,info.bitSize());
							else if (info.bitSize() == 32)
								slave.readPdo(info.index(),info.subindex(),&info.uint32,info.bitSize());
							else // 64
								slave.readPdo(info.index(),info.subindex(),&info.uint64,info.bitSize());
						}
						else
						{
							if (info.bitSize() == 8)
								slave.readPdo(info.index(),info.subindex(),&info.int8,info.bitSize());
							else if (info.bitSize() == 16)
								slave.readPdo(info.index(),info.subindex(),&info.int16,info.bitSize());
							else if (info.bitSize() == 32)
								slave.readPdo(info.index(),info.subindex(),&info.int32,info.bitSize());
							else // 64
								slave.readPdo(info.index(),info.subindex(),&info.int64,info.bitSize());
						}
					}

				}, param);
			}

			auto par_ = std::any_cast<GetSlaveParam&>(param);

			for (auto& info : par_.pdoInfoPool())
			{
				if (info.is_uint_)
				{
					if (info.bitSize() == 8)
						ret[info.name_] = info.uint8;
					else if (info.bitSize() == 16)
						ret[info.name_] = info.uint16;
					else if (info.bitSize() == 32)
						ret[info.name_] = info.uint32;
					else // 64
						ret[info.name_] = info.uint64;
				}
				else
				{
					if (info.bitSize() == 8)
						ret[info.name_] = info.int8;
					else if (info.bitSize() == 16)
						ret[info.name_] = info.int16;
					else if (info.bitSize() == 32)
						ret[info.name_] = info.int32;
					else // 64
						ret[info.name_] = info.int64;
				}
			}

			return ret;
		};
	
		for (auto& slave_param:paramPool())
		{
            ret[slave_param.name_] = func_get_slave(slave_param);
        }
        
        return ret;
    }, "user_slave_msg", kaanh::robot::Robot::GetType::kInfo);
}
auto UserSlave::execute(const std::string &str, std::function<void(std::string)> send_ret) noexcept->std::pair<std::string, std::string> {
    std::string_view cmd_name;
    std::map<std::string_view, std::string_view> params;
    
    try {
        std::tie(cmd_name, params) = imp_->parser_.parse(str);
    } catch (const std::exception &err) {
        sendRet(send_ret, -1, LOCALE_SELECT("invalid command '"+str+"'", "无效的指令'"+str+"'"));
        return {"", ""};
    }

    try {
        if (cmd_name == "setpdo" || cmd_name == "setpdo") {

            sendRet(send_ret, 0);
        } else {
            LOCALE_THROW("unrecognized command name '"+std::string(cmd_name)+"'", "无法识别的指令名称'"+std::string(cmd_name)+"'");
        }
    } catch (const std::exception &err) {
        sendRet(send_ret, -1, err.what());
    }
    END_CMD_FLOW;
}
auto UserSlave::instanceInCs()->UserSlave& {
    return dynamic_cast<UserSlave&>(kaanh::middleware::Shell::instanceInCs().getModule("UserSlave"));
}

ARIS_REGISTRATION {

    aris::core::class_<PdoEntryInfo>("PdoEntryInfo")
    .prop("name", &PdoEntryInfo::name_)
	.prop("is_uint", &PdoEntryInfo::is_uint_)
	.prop("index", &PdoEntryInfo::setIndex, &PdoEntryInfo::index)
	.propertyToStrMethod("index", Uint16ToHexStr)
	.prop("subindex", &PdoEntryInfo::setSubindex, &PdoEntryInfo::subindex)
	.propertyToStrMethod("subindex", Uint8ToHexStr)
	.prop("bitsize", &PdoEntryInfo::setBitSize, &PdoEntryInfo::bitSize);

	aris::core::class_<std::vector<PdoEntryInfo>>("PdoEntryInfoVec").asArray();

    aris::core::class_<GetSlaveParam>("GetSlaveParam")
    .prop("name", &GetSlaveParam::name_)
    .prop("slave_id", &GetSlaveParam::slave_id_)
	.prop("_pdo_info_vec", &GetSlaveParam::setPdoInfoPool,&GetSlaveParam::pdoInfoPool)
    ;


#define VECTOR_TO_STRING(set_name, get_name)  \
auto FUNC_SET_##set_name = [](UserSlave *obj, std::string str)->void {   \
     std::vector<int> v = nlohmann::json::parse(str).get<std::vector<int>>(); \
	obj->set_name(v);                                                       \
};                                                                                                                      \
auto FUNC_GET_##get_name = [](UserSlave *obj)->std::string{                                                           \
     return nlohmann::json(obj->get_name()).dump();                                       \
};

    VECTOR_TO_STRING(setSlaveIds,slaveIds)

	aris::core::class_<aris::core::PointerArray<GetSlaveParam>>("SlaveParamPool").asRefArray();

    typedef aris::core::PointerArray<GetSlaveParam>& (UserSlave::*GetSlaveParamPoolFunc)();
    aris::core::class_<UserSlave>("UserSlave").inherit<kaanh::module::MiddleModule>()
		.prop("io_ids", &FUNC_SET_setSlaveIds, &FUNC_GET_slaveIds)
		.prop("_slave_param_pool", &UserSlave::resetParamPool, (GetSlaveParamPoolFunc)(&UserSlave::paramPool));
}

}

