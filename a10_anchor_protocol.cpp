#include "a10_anchor_protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

#include "io/json.hpp"

namespace a10_tcp
{
namespace
{
bool fail(std::string* error, const std::string& message)
{
    if (error != nullptr)
    {
        *error = message;
    }
    return false;
}

bool parse_nonnegative_u64(
    const nlohmann::json& object, const char* key, std::uint64_t& value, std::string* error)
{
    if (!object.contains(key))
    {
        return fail(error, std::string("missing ") + key);
    }
    const auto& item = object[key];
    if (item.is_number_unsigned())
    {
        value = item.get<std::uint64_t>();
        return true;
    }
    if (item.is_number_integer())
    {
        const std::int64_t signed_value = item.get<std::int64_t>();
        if (signed_value >= 0)
        {
            value = static_cast<std::uint64_t>(signed_value);
            return true;
        }
    }
    return fail(error, std::string(key) + " must be a non-negative integer");
}

void rotvec_to_rotation(const double w[3], double rotation[9])
{
    const double theta = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (theta < 1e-12)
    {
        rotation[0] = 1.0;
        rotation[1] = 0.0;
        rotation[2] = 0.0;
        rotation[3] = 0.0;
        rotation[4] = 1.0;
        rotation[5] = 0.0;
        rotation[6] = 0.0;
        rotation[7] = 0.0;
        rotation[8] = 1.0;
        return;
    }

    const double x = w[0] / theta;
    const double y = w[1] / theta;
    const double z = w[2] / theta;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double one_minus_c = 1.0 - c;
    rotation[0] = c + x * x * one_minus_c;
    rotation[1] = x * y * one_minus_c - z * s;
    rotation[2] = x * z * one_minus_c + y * s;
    rotation[3] = y * x * one_minus_c + z * s;
    rotation[4] = c + y * y * one_minus_c;
    rotation[5] = y * z * one_minus_c - x * s;
    rotation[6] = z * x * one_minus_c - y * s;
    rotation[7] = z * y * one_minus_c + x * s;
    rotation[8] = c + z * z * one_minus_c;
}

void multiply_pm(const double left[16], const double right[16], double out[16])
{
    for (int row = 0; row < 4; ++row)
    {
        for (int column = 0; column < 4; ++column)
        {
            double value = 0.0;
            for (int k = 0; k < 4; ++k)
            {
                value += left[row * 4 + k] * right[k * 4 + column];
            }
            out[row * 4 + column] = value;
        }
    }
}

double vec3_norm(const double value[3])
{
    return std::sqrt(
        value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

void rotation_transpose_multiply(
    const double left_pm[16], const double right_pm[16], double rotation[9])
{
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            rotation[row * 3 + column] =
                left_pm[row] * right_pm[column]
                + left_pm[4 + row] * right_pm[4 + column]
                + left_pm[8 + row] * right_pm[8 + column];
        }
    }
}

double rotation_angle(const double left_pm[16], const double right_pm[16])
{
    double relative[9]{};
    rotation_transpose_multiply(left_pm, right_pm, relative);
    const double cosine = std::clamp(
        0.5 * (relative[0] + relative[4] + relative[8] - 1.0), -1.0, 1.0);
    return std::acos(cosine);
}

void rotation_matrix_to_rotvec(const double rotation[9], double rotvec[3])
{
    const double cosine = std::clamp(
        0.5 * (rotation[0] + rotation[4] + rotation[8] - 1.0), -1.0, 1.0);
    const double angle = std::acos(cosine);
    if (angle < 1e-12)
    {
        rotvec[0] = 0.0;
        rotvec[1] = 0.0;
        rotvec[2] = 0.0;
        return;
    }

    const double sine = std::sin(angle);
    if (std::abs(sine) > 1e-8)
    {
        const double scale = angle / (2.0 * sine);
        rotvec[0] = (rotation[7] - rotation[5]) * scale;
        rotvec[1] = (rotation[2] - rotation[6]) * scale;
        rotvec[2] = (rotation[3] - rotation[1]) * scale;
        return;
    }

    // Stable axis extraction close to pi, including mixed-sign axes.
    const double diagonal[3] = {
        std::max(0.0, 0.5 * (rotation[0] + 1.0)),
        std::max(0.0, 0.5 * (rotation[4] + 1.0)),
        std::max(0.0, 0.5 * (rotation[8] + 1.0)),
    };
    double axis[3]{};
    if (diagonal[0] >= diagonal[1] && diagonal[0] >= diagonal[2])
    {
        axis[0] = std::sqrt(diagonal[0]);
        const double divisor = std::max(4.0 * axis[0], 1e-12);
        axis[1] = (rotation[1] + rotation[3]) / divisor;
        axis[2] = (rotation[2] + rotation[6]) / divisor;
    }
    else if (diagonal[1] >= diagonal[2])
    {
        axis[1] = std::sqrt(diagonal[1]);
        const double divisor = std::max(4.0 * axis[1], 1e-12);
        axis[0] = (rotation[1] + rotation[3]) / divisor;
        axis[2] = (rotation[5] + rotation[7]) / divisor;
    }
    else
    {
        axis[2] = std::sqrt(diagonal[2]);
        const double divisor = std::max(4.0 * axis[2], 1e-12);
        axis[0] = (rotation[2] + rotation[6]) / divisor;
        axis[1] = (rotation[5] + rotation[7]) / divisor;
    }
    const double norm = vec3_norm(axis);
    if (norm < 1e-12)
    {
        axis[0] = 1.0;
        axis[1] = 0.0;
        axis[2] = 0.0;
    }
    else
    {
        axis[0] /= norm;
        axis[1] /= norm;
        axis[2] /= norm;
    }
    rotvec[0] = axis[0] * angle;
    rotvec[1] = axis[1] * angle;
    rotvec[2] = axis[2] * angle;
}
}  // namespace

const char* anchor_control_state_name(AnchorControlState state)
{
    switch (state)
    {
    case AnchorControlState::inactive: return "inactive";
    case AnchorControlState::tracking: return "tracking";
    case AnchorControlState::frozen: return "frozen";
    case AnchorControlState::fault: return "fault";
    }
    return "unknown";
}

double slew_toward(double current, double target, double max_delta)
{
    const double bounded_delta = std::max(0.0, max_delta);
    return current + std::clamp(target - current, -bounded_delta, bounded_delta);
}

double effective_anchor_speed_limit(
    double controller_speed_limit, double reference_speed_limit)
{
    return std::max(
        0.0, std::min(controller_speed_limit, reference_speed_limit));
}

void EeAnchorReferenceGovernor::set_config(const AnchorGovernorConfig& config)
{
    config_ = config;
    config_.max_reference_linear_speed_m_s =
        std::max(0.0, config_.max_reference_linear_speed_m_s);
    config_.max_reference_angular_speed_rad_s =
        std::max(0.0, config_.max_reference_angular_speed_rad_s);
    config_.max_tracking_error_m = std::max(0.0, config_.max_tracking_error_m);
    config_.max_tracking_error_rad = std::max(0.0, config_.max_tracking_error_rad);
    config_.fault_after_frozen_cycles =
        std::max<std::uint32_t>(1, config_.fault_after_frozen_cycles);
}

void EeAnchorReferenceGovernor::set_reference(const double pm[16])
{
    std::copy_n(pm, reference_pm_.size(), reference_pm_.begin());
}

void EeAnchorReferenceGovernor::reset()
{
    state_ = AnchorControlState::inactive;
    reference_pm_.fill(0.0);
    tracking_error_m_ = 0.0;
    tracking_error_rad_ = 0.0;
    frozen_cycles_ = 0;
}

void EeAnchorReferenceGovernor::engage(const double actual_pm[16])
{
    set_reference(actual_pm);
    state_ = AnchorControlState::tracking;
    tracking_error_m_ = 0.0;
    tracking_error_rad_ = 0.0;
    frozen_cycles_ = 0;
}

void EeAnchorReferenceGovernor::release(const double actual_pm[16])
{
    set_reference(actual_pm);
    state_ = AnchorControlState::inactive;
    tracking_error_m_ = 0.0;
    tracking_error_rad_ = 0.0;
    frozen_cycles_ = 0;
}

void EeAnchorReferenceGovernor::force_fault(const double actual_pm[16])
{
    set_reference(actual_pm);
    state_ = AnchorControlState::fault;
    frozen_cycles_ = 0;
}

AnchorControlState EeAnchorReferenceGovernor::step(
    const double user_target_pm[16], const double actual_pm[16], double dt_s,
    bool hold_reference)
{
    if (!control_active() || dt_s <= 0.0)
    {
        return state_;
    }

    const double tracking_translation[3] = {
        reference_pm_[3] - actual_pm[3],
        reference_pm_[7] - actual_pm[7],
        reference_pm_[11] - actual_pm[11],
    };
    tracking_error_m_ = vec3_norm(tracking_translation);
    tracking_error_rad_ = rotation_angle(reference_pm_.data(), actual_pm);
    if (tracking_error_m_ > config_.max_tracking_error_m
        || tracking_error_rad_ > config_.max_tracking_error_rad)
    {
        ++frozen_cycles_;
        state_ = frozen_cycles_ >= config_.fault_after_frozen_cycles
            ? AnchorControlState::fault
            : AnchorControlState::frozen;
        if (state_ == AnchorControlState::fault)
        {
            set_reference(actual_pm);
        }
        return state_;
    }

    frozen_cycles_ = 0;
    if (hold_reference)
    {
        // Joint limiting pauses reference advance, but never skips feedback
        // monitoring or hides a persistent Cartesian tracking error above.
        state_ = AnchorControlState::frozen;
        return state_;
    }
    state_ = AnchorControlState::tracking;

    double translation_step[3] = {
        user_target_pm[3] - reference_pm_[3],
        user_target_pm[7] - reference_pm_[7],
        user_target_pm[11] - reference_pm_[11],
    };
    const double translation_norm = vec3_norm(translation_step);
    const double max_translation_step = config_.max_reference_linear_speed_m_s * dt_s;
    if (translation_norm > max_translation_step && translation_norm > 1e-12)
    {
        const double scale = max_translation_step / translation_norm;
        translation_step[0] *= scale;
        translation_step[1] *= scale;
        translation_step[2] *= scale;
    }

    double relative_rotation[9]{};
    rotation_transpose_multiply(reference_pm_.data(), user_target_pm, relative_rotation);
    double rotation_step[3]{};
    rotation_matrix_to_rotvec(relative_rotation, rotation_step);
    const double rotation_norm = vec3_norm(rotation_step);
    const double max_rotation_step = config_.max_reference_angular_speed_rad_s * dt_s;
    if (rotation_norm > max_rotation_step && rotation_norm > 1e-12)
    {
        const double scale = max_rotation_step / rotation_norm;
        rotation_step[0] *= scale;
        rotation_step[1] *= scale;
        rotation_step[2] *= scale;
    }

    double incremental_rotation[9]{};
    rotvec_to_rotation(rotation_step, incremental_rotation);
    const double rotation_increment_pm[16] = {
        incremental_rotation[0], incremental_rotation[1], incremental_rotation[2], 0.0,
        incremental_rotation[3], incremental_rotation[4], incremental_rotation[5], 0.0,
        incremental_rotation[6], incremental_rotation[7], incremental_rotation[8], 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    double advanced[16]{};
    multiply_pm(reference_pm_.data(), rotation_increment_pm, advanced);
    advanced[3] = reference_pm_[3] + translation_step[0];
    advanced[7] = reference_pm_[7] + translation_step[1];
    advanced[11] = reference_pm_[11] + translation_step[2];
    set_reference(advanced);
    return state_;
}

bool parse_ee_anchor_line(const std::string& line, EeAnchorCommand& out, std::string* error)
{
    const std::size_t json_start = line.find('{');
    if (json_start == std::string::npos)
    {
        return fail(error, "missing JSON object");
    }
    try
    {
        const nlohmann::json object = nlohmann::json::parse(line.substr(json_start));
        const std::string prefix = "SET_EE_ANCHOR";
        const bool prefix_boundary =
            line.size() == prefix.size()
            || (line.size() > prefix.size()
                && (std::isspace(static_cast<unsigned char>(line[prefix.size()]))
                    || line[prefix.size()] == '{'));
        const bool prefixed = line.compare(0, prefix.size(), prefix) == 0 && prefix_boundary;
        const bool typed =
            object.contains("type") && object["type"].is_string()
            && object["type"].get<std::string>() == "SET_EE_ANCHOR";
        if (!prefixed && !typed)
        {
            return fail(error, "not a SET_EE_ANCHOR message");
        }
        if (object.contains("type") && !typed)
        {
            return fail(error, "invalid type");
        }
        if (!object.contains("active") || !object["active"].is_boolean())
        {
            return fail(error, "active must be boolean");
        }
        if (!object.contains("session_id") || !object["session_id"].is_string())
        {
            return fail(error, "session_id must be string");
        }

        EeAnchorCommand parsed;
        parsed.active = object["active"].get<bool>();
        parsed.session_id = object["session_id"].get<std::string>();
        if (parsed.session_id.empty())
        {
            return fail(error, "session_id cannot be empty");
        }
        if (!parse_nonnegative_u64(object, "anchor_id", parsed.anchor_id, error)
            || !parse_nonnegative_u64(object, "sample_sequence", parsed.sample_sequence, error))
        {
            return false;
        }
        if (!object.contains("offset") || !object["offset"].is_array()
            || object["offset"].size() != parsed.offset.size())
        {
            return fail(error, "offset must contain exactly 6 numbers");
        }
        for (std::size_t i = 0; i < parsed.offset.size(); ++i)
        {
            const auto& item = object["offset"][i];
            if (!item.is_number())
            {
                return fail(error, "offset must contain exactly 6 numbers");
            }
            parsed.offset[i] = item.get<double>();
            if (!std::isfinite(parsed.offset[i]))
            {
                return fail(error, "offset values must be finite");
            }
        }
        if (!object.contains("gripper") || !object["gripper"].is_number())
        {
            return fail(error, "gripper must be numeric");
        }
        parsed.gripper = object["gripper"].get<double>();
        if (!std::isfinite(parsed.gripper))
        {
            return fail(error, "gripper must be finite");
        }
        parsed.has_client_sample_time = object.contains("client_sample_time_ns");
        parsed.has_client_send_time = object.contains("client_send_time_ns");
        if ((parsed.has_client_sample_time && !parse_nonnegative_u64(
                object, "client_sample_time_ns", parsed.client_sample_time_ns, error))
            || (parsed.has_client_send_time && !parse_nonnegative_u64(
                object, "client_send_time_ns", parsed.client_send_time_ns, error)))
            return false;
        if (parsed.has_client_sample_time && parsed.has_client_send_time
            && parsed.client_send_time_ns < parsed.client_sample_time_ns)
            return fail(error, "client_send_time_ns must not precede client_sample_time_ns");
        out = std::move(parsed);
        if (error != nullptr)
        {
            error->clear();
        }
        return true;
    }
    catch (const std::exception& exception)
    {
        return fail(error, exception.what());
    }
}

AnchorShadowUpdate EeAnchorShadow::update(
    const EeAnchorCommand& command, const double actual_pm[16])
{
    if (!command.active)
    {
        active_ = false;
        session_id_ = command.session_id;
        anchor_id_ = command.anchor_id;
        sample_sequence_ = command.sample_sequence;
        return AnchorShadowUpdate::inactive;
    }

    const bool new_session = !active_ || command.session_id != session_id_;
    if (!new_session && command.anchor_id < anchor_id_)
    {
        return AnchorShadowUpdate::stale;
    }
    const bool new_anchor = new_session || command.anchor_id > anchor_id_;
    if (!new_anchor)
    {
        if (command.sample_sequence < sample_sequence_)
        {
            return AnchorShadowUpdate::stale;
        }
        if (command.sample_sequence == sample_sequence_)
        {
            return AnchorShadowUpdate::duplicate;
        }
    }

    if (new_anchor)
    {
        std::copy_n(actual_pm, robot_anchor_pm_.size(), robot_anchor_pm_.begin());
    }
    active_ = true;
    session_id_ = command.session_id;
    anchor_id_ = command.anchor_id;
    sample_sequence_ = command.sample_sequence;
    update_user_target(command.offset);
    return new_anchor ? AnchorShadowUpdate::anchored : AnchorShadowUpdate::updated;
}

void EeAnchorShadow::reset()
{
    active_ = false;
    session_id_.clear();
    anchor_id_ = 0;
    sample_sequence_ = 0;
    robot_anchor_pm_.fill(0.0);
    user_target_pm_.fill(0.0);
}

void EeAnchorShadow::update_user_target(const std::array<double, 6>& offset)
{
    double rotation[9]{};
    rotvec_to_rotation(offset.data() + 3, rotation);
    const double offset_pm[16] = {
        rotation[0], rotation[1], rotation[2], offset[0],
        rotation[3], rotation[4], rotation[5], offset[1],
        rotation[6], rotation[7], rotation[8], offset[2],
        0.0, 0.0, 0.0, 1.0,
    };
    multiply_pm(robot_anchor_pm_.data(), offset_pm, user_target_pm_.data());
}
}  // namespace a10_tcp
