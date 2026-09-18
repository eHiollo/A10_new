#ifndef A10_ANCHOR_PROTOCOL_HPP_
#define A10_ANCHOR_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace a10_tcp
{
struct EeAnchorCommand
{
    bool active{false};
    std::string session_id;
    std::uint64_t anchor_id{0};
    std::uint64_t sample_sequence{0};
    std::array<double, 6> offset{};
    double gripper{0.0};
};

bool parse_ee_anchor_line(
    const std::string& line, EeAnchorCommand& out, std::string* error = nullptr);

enum class AnchorShadowUpdate
{
    inactive,
    anchored,
    updated,
    duplicate,
    stale,
};

/// A2.2 diagnostic-only robot/user anchor state. It never returns a motor command.
class EeAnchorShadow
{
public:
    AnchorShadowUpdate update(const EeAnchorCommand& command, const double actual_pm[16]);
    void reset();

    bool active() const { return active_; }
    const std::string& session_id() const { return session_id_; }
    std::uint64_t anchor_id() const { return anchor_id_; }
    std::uint64_t sample_sequence() const { return sample_sequence_; }
    const std::array<double, 16>& robot_anchor_pm() const { return robot_anchor_pm_; }
    const std::array<double, 16>& user_target_pm() const { return user_target_pm_; }

private:
    void update_user_target(const std::array<double, 6>& offset);

    bool active_{false};
    std::string session_id_;
    std::uint64_t anchor_id_{0};
    std::uint64_t sample_sequence_{0};
    std::array<double, 16> robot_anchor_pm_{};
    std::array<double, 16> user_target_pm_{};
};
}  // namespace a10_tcp

#endif  // A10_ANCHOR_PROTOCOL_HPP_
