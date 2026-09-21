// Recorded-command envelope replay, NOT a counterfactual IK/servo simulation.
// Usage: executable path/to/vr_joint_trace.txt [first_count]
#include "a10_vr_joint_guard.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

static std::vector<std::string> split(const std::string& line)
{
    std::vector<std::string> fields;
    std::istringstream in(line);
    for (std::string value; std::getline(in, value, ',');) fields.push_back(value);
    return fields;
}

int main(int argc, char** argv)
{
    using Guard = a10_tcp::VrJointGuard;
    if (argc < 2) return 2;
    std::ifstream input(argv[1]);
    if (!input) throw std::runtime_error("cannot open trace");
    std::string line;
    std::getline(input, line);
    const auto header = split(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
    const long start = argc > 2 ? std::stol(argv[2]) : 1;
    Guard guard;
    Guard::Joints q{}, v{}, peak_v{}, peak_a{};
    bool initialized = false;
    long rows = 0, first_fault = 0, limited = 0;
    for (; std::getline(input, line);)
    {
        const auto fields = split(line);
        if (fields.size() != header.size()) throw std::runtime_error("malformed trace row");
        auto get = [&](const std::string& name) { return std::stod(fields.at(columns.at(name))); };
        const long count = static_cast<long>(get("count"));
        if (count < start) continue;
        Guard::Joints desired{}, actual{};
        for (std::size_t i = 0; i < 6; ++i)
        {
            const auto prefix = "j" + std::to_string(i + 1);
            if (!initialized) q[i] = get(prefix + "_q_prev");
            // Apply the recorded requested *step* to the guarded previous command.
            desired[i] = q[i] + get(prefix + "_dq");
            actual[i] = get(prefix + "_q_actual");
        }
        initialized = true;
        const auto result = guard.step(q, v, desired, actual);
        if (guard.faulted() && !first_fault) first_fault = count;
        limited += result.limited;
        for (std::size_t i = 0; i < 6; ++i)
        {
            const double speed = (result.position[i] - q[i]) / Guard::dt;
            const double accel = (speed - v[i]) / Guard::dt;
            assert(std::isfinite(speed));
            assert(std::abs(speed) <= Guard::speed_limit(i) + 1e-8);
            assert(std::abs(accel) <= Guard::acceleration + 1e-6);
            assert(std::abs(result.position[i]) <= Guard::position_limit + 1e-8);
            peak_v[i] = std::max(peak_v[i], std::abs(speed));
            peak_a[i] = std::max(peak_a[i], std::abs(accel));
            v[i] = speed;
        }
        q = result.position;
        ++rows;
    }
    assert(rows > 0 && first_fault > 0);
    std::cout << "PASS rows=" << rows << " first_fault=" << first_fault
              << " reason=" << guard.fault() << " limited=" << limited << '\n';
    for (std::size_t i = 0; i < 6; ++i)
        std::cout << "J" << i + 1 << " peak_speed=" << peak_v[i]
                  << " peak_accel=" << peak_a[i] << '\n';
}
