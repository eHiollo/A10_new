#pragma once
#include <filesystem>
#include <cctype>
#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "kaanh/general/json.hpp"
#include "a10_follower_core.hpp"

namespace a10_follower {
inline std::string command_body(std::string body) {
    while (!body.empty() && (body.back() == '\0' || std::isspace(static_cast<unsigned char>(body.back())))) body.pop_back();
    const auto start = body.find_first_not_of(" \t\r\n");
    return start == std::string::npos ? std::string{} : body.substr(start);
}
inline Joints parse_jp(std::string body) {
    body = command_body(std::move(body));
    const auto space = body.find_first_of(" \t\r\n");
    if (space == std::string::npos || body.substr(0, space) != "jp") throw std::invalid_argument("expected jp");
    body = command_body(body.substr(space));
    const std::string prefix = "--motor_pos=";
    if (body.compare(0, prefix.size(), prefix) != 0) throw std::invalid_argument("expected --motor_pos=[six degrees]");
    auto values = nlohmann::json::parse(body.substr(prefix.size()));
    if (!values.is_array() || values.size() != 6) throw std::invalid_argument("jp requires exactly six joints");
    Joints result{};
    constexpr double rad = 3.14159265358979323846 / 180;
    for (std::size_t i = 0; i < 6; ++i) {
        if (!values[i].is_number()) throw std::invalid_argument("jp requires numeric joint positions");
        result[i] = values[i].get<double>() * rad;
        if (!std::isfinite(result[i])) throw std::invalid_argument("jp requires finite joints");
    }
    return result;
}
struct ConfigUpdate { Limits limits; bool save{false}, changed{false}; };
inline ConfigUpdate parse_config(const std::string& body, const Limits& current, bool allow_save = true) {
    ConfigUpdate update{current}; std::istringstream input(command_body(body)); std::string token;
    input >> token; std::map<std::string, bool> seen;
    while (input >> token) {
        const auto equal = token.find('=');
        if (equal == std::string::npos || token.substr(0, 2) != "--") throw std::invalid_argument("use --parameter=value");
        const auto key = token.substr(2, equal - 2), value = token.substr(equal + 1);
        if (!seen.emplace(key, true).second) throw std::invalid_argument("duplicate follower parameter");
        if (key == "save" && allow_save) {
            if (value != "0" && value != "1") throw std::invalid_argument("save must be 0 or 1");
            update.save = value == "1"; continue;
        }
        double* destination = key == "max_vel" ? &update.limits.max_vel : key == "max_acc" ? &update.limits.max_acc : key == "max_dec" ? &update.limits.max_dec : nullptr;
        if (!destination) throw std::invalid_argument("unsupported follower parameter: " + key);
        std::size_t consumed{}; *destination = std::stod(value, &consumed);
        if (consumed != value.size()) throw std::invalid_argument("invalid follower number");
        update.changed = true;
    }
    update.limits.validate(); return update;
}
inline nlohmann::json limits_json(const Limits& limits) {
    return {{"max_vel",limits.max_vel},{"max_acc",limits.max_acc},{"max_dec",limits.max_dec}};
}
inline Limits load_limits(const std::filesystem::path& path, Limits defaults) {
    if (!std::filesystem::exists(path)) { defaults.validate(); return defaults; }
    std::ifstream file(path); if (!file) throw std::runtime_error("cannot read follower_config.json");
    nlohmann::json value; file >> value;
    if (!file.eof()) { file >> std::ws; if (!file.eof()) throw std::invalid_argument("trailing follower configuration data"); }
    if (!value.is_object() || value.size() != 4 || value.at("version") != 1
        || !value.at("max_vel").is_number() || !value.at("max_acc").is_number() || !value.at("max_dec").is_number())
        throw std::invalid_argument("invalid follower_config.json schema");
    Limits result{value.at("max_vel").get<double>(),value.at("max_acc").get<double>(),value.at("max_dec").get<double>()};
    result.validate(); return result;
}
inline void save_limits(const std::filesystem::path& path, const Limits& limits) {
    limits.validate(); auto json = limits_json(limits); json["version"] = 1;
    // Called under the module's NRT configuration mutex. Never truncate the
    // deployed file: flush/close and rename a sibling temporary file instead.
    auto temporary = path; temporary += ".tmp";
    try {
        std::ofstream file(temporary, std::ios::trunc);
        if (!file) throw std::runtime_error("cannot create follower config temporary file");
        file << json.dump(2) << '\n'; file.flush();
        if (!file) throw std::runtime_error("cannot write follower config");
        file.close(); if (!file) throw std::runtime_error("cannot close follower config");
        #ifdef __linux__
        const int fd = ::open(temporary.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) throw std::runtime_error("cannot open follower config for sync");
        const int synced = ::fsync(fd); ::close(fd);
        if (synced != 0) throw std::runtime_error("cannot sync follower config");
#endif
        std::filesystem::rename(temporary, path);
#ifdef __linux__
        const auto parent = path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path();
        const int directory = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
        if (directory >= 0) { ::fsync(directory); ::close(directory); }
#endif
    } catch (...) { std::error_code ec; std::filesystem::remove(temporary, ec); throw; }
}
} // namespace a10_follower
