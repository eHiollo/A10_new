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

enum class AnchorControlState
{
    inactive,
    tracking,
    frozen,
    fault,
};

const char* anchor_control_state_name(AnchorControlState state);

/// Shared shaping helpers used by A2.3 and covered without the robot SDK.
double slew_toward(double current, double target, double max_delta);
double effective_anchor_speed_limit(
    double controller_speed_limit, double reference_speed_limit);

struct AnchorGovernorConfig
{
    double max_reference_linear_speed_m_s{0.06};
    double max_reference_angular_speed_rad_s{0.25};
    double max_tracking_error_m{0.05};
    double max_tracking_error_rad{0.35};
    std::uint32_t fault_after_frozen_cycles{50};
};

/// A2.3 bounded reference generator. It owns only the Cartesian reference;
/// the existing vr_vel P/speed/slew/IK chain still owns the motor command.
class EeAnchorReferenceGovernor
{
public:
    void set_config(const AnchorGovernorConfig& config);
    void reset();
    void engage(const double actual_pm[16]);
    void release(const double actual_pm[16]);
    void force_fault(const double actual_pm[16]);
    AnchorControlState step(
        const double user_target_pm[16], const double actual_pm[16], double dt_s);

    AnchorControlState state() const { return state_; }
    bool control_active() const
    {
        return state_ == AnchorControlState::tracking || state_ == AnchorControlState::frozen;
    }
    const std::array<double, 16>& reference_pm() const { return reference_pm_; }
    double tracking_error_m() const { return tracking_error_m_; }
    double tracking_error_rad() const { return tracking_error_rad_; }
    std::uint32_t frozen_cycles() const { return frozen_cycles_; }

private:
    void set_reference(const double pm[16]);

    AnchorGovernorConfig config_;
    AnchorControlState state_{AnchorControlState::inactive};
    std::array<double, 16> reference_pm_{};
    double tracking_error_m_{0.0};
    double tracking_error_rad_{0.0};
    std::uint32_t frozen_cycles_{0};
};

/// A2.2/A2.3 robot/user anchor state. Motor ownership remains in vr_vel.
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
