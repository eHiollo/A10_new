#ifndef A10_INIT_PRESETS_HPP_
#define A10_INIT_PRESETS_HPP_

namespace a10_init
{
constexpr int k_preset_count = 6;
constexpr int k_preset_joint_num = 12;
constexpr double k_home_step_rad = 0.0001;
constexpr double k_home_tol_rad = 0.00009;
constexpr int k_home_timeout_cycles = 28000;

inline int clamp_preset(int index)
{
    if (index < 0 || index >= k_preset_count)
    {
        return 0;
    }
    return index;
}

/// 六组初始关节目标（rad），每组 12 维：臂1 前 6 + 臂2 后 6。与 ``m_init --preset`` 共用。
inline const double* preset_joints(int index)
{
    constexpr double k_pi = 3.14159265358979323846;
    constexpr double k_deg = k_pi / 180.0;
    static constexpr double k_presets[k_preset_count][k_preset_joint_num] = {
        {
            -14.0 * k_deg, -24.0 * k_deg, 139.0 * k_deg, -77.0 * k_deg, -76.0 * k_deg, -8.0 * k_deg,
            0.0, 0.0, -5.0 * k_pi / 6.0, 5.0 * k_pi / 6.0, k_pi / 2.0, 0.0,
        },
        {
            -19.5 * k_deg, -19.3 * k_deg, 126.5 * k_deg, -61.0 * k_deg, -70.0 * k_deg, -9.0 * k_deg,
            0.0, 0.0, -5.0 * k_pi / 6.0, 5.0 * k_pi / 6.0, k_pi / 2.0, 0.0,
        },
        {
            -31.0 * k_deg, -23.5 * k_deg, 123.5 * k_deg, -51.6 * k_deg, -62.4 * k_deg, -22.0 * k_deg,
            0.0, 0.0, -5.0 * k_pi / 6.0, 5.0 * k_pi / 6.0, k_pi / 2.0, 0.0,
        },
        {
            -40.0 * k_deg, -8.8 * k_deg, 106.5 * k_deg, -32.8 * k_deg, -54.3 * k_deg, -33.0 * k_deg,
            0.0, 0.0, -5.0 * k_pi / 6.0, 5.0 * k_pi / 6.0, k_pi / 2.0, 0.0,
        },
        {
            -11.7 * k_deg, -7.7 * k_deg, 95.6 * k_deg, -15.8 * k_deg, -86.3 * k_deg, -2.5 * k_deg,
            0.0, 0.0, -5.0 * k_pi / 6.0, 5.0 * k_pi / 6.0, k_pi / 2.0, 0.0,
        },
        {
            -7.8 * k_deg, 1.6 * k_deg, 89.7 * k_deg, -18.6 * k_deg, -88.7 * k_deg, 0.9 * k_deg,
            0.0, 0.0, -5.0 * k_pi / 6.0, 5.0 * k_pi / 6.0, k_pi / 2.0, 0.0,
        },
    };
    return k_presets[clamp_preset(index)];
}
}  // namespace a10_init

#endif
