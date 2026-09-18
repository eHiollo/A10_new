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
}  // namespace

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
