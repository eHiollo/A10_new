#ifndef A10_VR_RESET_COMMAND_HPP_
#define A10_VR_RESET_COMMAND_HPP_

#include <cctype>
#include <string>
#include <string_view>

namespace a10_tcp
{
/// 8080 遥操口上的复位命令。与 Shell 的 ``reset`` 同名，大小写不敏感。
inline bool is_vr_reset_command(std::string_view line)
{
    std::size_t begin = 0;
    std::size_t end = line.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(line[begin])))
    {
        ++begin;
    }
    while (end > begin && std::isspace(static_cast<unsigned char>(line[end - 1])))
    {
        --end;
    }
    constexpr std::string_view k_reset = "reset";
    if (end - begin != k_reset.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < k_reset.size(); ++i)
    {
        const auto ch = static_cast<unsigned char>(line[begin + i]);
        if (std::tolower(ch) != static_cast<unsigned char>(k_reset[i]))
        {
            return false;
        }
    }
    return true;
}
}  // namespace a10_tcp

#endif
