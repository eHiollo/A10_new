#ifndef A10_STARTUP_LOGIN_HPP_
#define A10_STARTUP_LOGIN_HPP_

#include "kaanh/general/md5.hpp"
#include "kaanhbot/user/user_management.hpp"

#include <ctime>
#include <ostream>
#include <stdexcept>
#include <string>

namespace a10_auth {

// Use the controller process's own local clock, just like the vendor SDK.
inline std::string manufacturer_password(std::time_t now)
{
    std::tm local{};
#ifdef _WIN32
    if (localtime_s(&local, &now) != 0)
#else
    if (localtime_r(&now, &local) == nullptr)
#endif
    {
        throw std::runtime_error("cannot read controller local time");
    }
    const int sum = local.tm_year + 1900 + local.tm_mon + 1 + local.tm_mday;
    return kaanh::general::md5(std::to_string(sum));
}

// Call after cs.init(), before starting command interfaces. Never echo the
// credential or exception text: SDK errors may contain sensitive arguments.
inline bool auto_login_manufacturer(std::ostream& out, std::ostream& err)
{
    using kaanhbot::user::User;
    try
    {
        auto& users = kaanhbot::user::UserManagement::instanceInCs();
        bool sdk_reported_error = false;
        if (users.user() == User::kNone)
        {
            const auto now = std::time(nullptr);
            if (now == static_cast<std::time_t>(-1))
            {
                throw std::runtime_error("cannot read controller clock");
            }
            try
            {
                users.login(User::kManufacturer, manufacturer_password(now));
            }
            catch (...)
            {
                // The SDK sets its role before writing the login audit record.
                // Inspect the final role even if that later operation fails.
                sdk_reported_error = true;
            }
        }
        if (users.user() == User::kManufacturer)
        {
            out << "自动登录成功：当前用户为 Manufacturer（厂家）。" << std::endl;
            if (sdk_reported_error)
            {
                err << "登录已生效，但 SDK 后续处理出现异常，请检查运行日志配置。" << std::endl;
            }
            return true;
        }
    }
    catch (...)
    {
        // Leave the command interface available for manual login/recovery.
    }
    err << "自动登录失败：请检查控制器日期和用户配置，或手动执行 login。" << std::endl;
    return false;
}

} // namespace a10_auth

#endif // A10_STARTUP_LOGIN_HPP_
