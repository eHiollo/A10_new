#ifndef KAANHBOT_USER_SLAVE_HPP_
#define KAANHBOT_USER_SLAVE_HPP_

#include <memory>

#include "kaanh/module/middle_module.hpp"
#include "kaanh/general/macro.hpp"
#include "kaanh/general/json.hpp"
#include "kaanhbot_lib_export.h"

namespace kaanhbot::user {

class PdoEntryInfo
{
public:

    std::string name_ = "name";
    bool is_uint_ = true;

    std::uint8_t uint8;
    std::uint16_t uint16;
    std::uint32_t uint32;
    std::uint64_t uint64;

    std::int8_t int8;
    std::int16_t int16;
    std::int32_t int32;
    std::int64_t int64;

private:
    std::uint16_t index_;
    std::uint8_t subindex_;
    aris::Size bitsize_;
public:


    auto setIndex(std::uint16_t index)->void;
    auto index()const->std::uint16_t;
    auto setSubindex(std::uint8_t subindex)->void;
    auto subindex()const->std::uint8_t;
    auto setBitSize(aris::Size bitsize)->void;
    auto bitSize()const->aris::Size;

    auto check() const ->void 
    {

        // if (trig_time < 0) { LOCALE_THROW("Trig time is can not small 0","触发时间不能小于0");};
        // if (trig_type == CameraTrigType::kNone) { LOCALE_THROW("Trig type is none","未知的触发类型");};
        // if (connect_type == CameraConnectType::kModbusTcpServer) { LOCALE_THROW("Connect type is none","未知的连接类型");};
        // if (trig_distance < 0) LOCALE_THROW("Trig distance is can not small 0", "触发距离不能小于0");
    }
public:
    PdoEntryInfo(/* args */) = default;
    ~PdoEntryInfo() = default;
};




class GetSlaveParam
{

public:
    GetSlaveParam(/* args */);
    ~GetSlaveParam();

    auto pdoInfoPool()->std::vector<PdoEntryInfo>&;
    auto setPdoInfoPool(const std::vector<PdoEntryInfo> &info)->void;

    // 全局参数
    std::vector<PdoEntryInfo> pdo_info_;
    std::string name_;
    std::uint16_t slave_id_ = -1;
private:
    // struct Imp;
    // std::unique_ptr<Imp> imp_;
 
};

class KAANHBOT_API UserSlave : public kaanh::module::MiddleModule {
public:
    auto virtual init()->void override;
    auto virtual execute(const std::string &str, std::function<void(std::string)> send_ret) noexcept->std::pair<std::string, std::string> override;
    auto setSlaveIds(const std::vector<int> slave_ids)->void;
    auto slaveIds()->const std::vector<int>;

    auto paramPool()->aris::core::PointerArray<GetSlaveParam>&;
    auto paramPool() const->const aris::core::PointerArray<GetSlaveParam>& { return const_cast<UserSlave*>(this)->paramPool(); }
    auto resetParamPool(aris::core::PointerArray<GetSlaveParam> *pool)->void;

    static auto instanceInCs()->UserSlave&;
    
private:
    struct Imp;
    std::unique_ptr<Imp> imp_;

public:
    UserSlave();
    virtual ~UserSlave();
    KAANH_DELETE_BIG_FOUR(UserSlave)
};

}   // namespace kaanhbot::user

#endif // KAANHBOT_USER_USER_MANAGEMENT_HPP_

