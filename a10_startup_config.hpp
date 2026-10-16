#ifndef A10_STARTUP_CONFIG_HPP_
#define A10_STARTUP_CONFIG_HPP_

#include <filesystem>
#include <stdexcept>
#include <string>

namespace a10_startup {

inline std::filesystem::path executable_directory(const char* argv0)
{
#ifdef __linux__
    std::error_code ec;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) throw std::runtime_error("cannot resolve executable path: " + ec.message());
    return executable.parent_path();
#else
    return std::filesystem::absolute(argv0).parent_path();
#endif
}

// Never search the source tree or parent folders: a missing runtime XML must
// produce an error instead of silently selecting another robot configuration.
inline std::filesystem::path xml_path(
    const std::filesystem::path& executable_dir,
    const std::filesystem::path& explicit_path = {})
{
    const auto path = std::filesystem::absolute(
        explicit_path.empty() ? executable_dir / "kaanh.xml" : explicit_path);
    std::error_code ec;
    const bool regular = std::filesystem::is_regular_file(path, ec);
    if (ec || !regular)
        throw std::runtime_error("runtime XML missing or unreadable: " + path.string());
    const auto size = std::filesystem::file_size(path, ec);
    if (ec || size == 0)
        throw std::runtime_error("runtime XML empty or unreadable: " + path.string());
    return path;
}

} // namespace a10_startup
#endif
