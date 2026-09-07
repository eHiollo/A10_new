/**
 * @file admittance_controller.hpp
 * @brief 导纳控制器接口定义 (重构版 - 使用统一数学库)
 */

#ifndef RTB_ADMITTANCE_CONTROLLER_HPP
#define RTB_ADMITTANCE_CONTROLLER_HPP

namespace rtb {
namespace force {

class AdmittanceController {
public:
    AdmittanceController();
    ~AdmittanceController() = default;

    // 重置积分状态
    auto reset() -> void ;

    // 设置阻抗参数 (Mass, Damping, Stiffness)
    // m, d, k 均为 6维数组 [x, y, z, rx, ry, rz]
    auto setParams(const double* m, const double* d, const double* k) -> void ;

    /**
     * @param tau 前馈时滞常数 (s)
     */
    auto setLagTau(double tau) -> void ;

    /** 设置控制周期 dt (s)，用于积分步长 */
    auto setDt(double dt) -> void ;

    /**
     * @brief 核心更新函数
     * @param cur_ft  传感器力 [fx,fy,fz, tx,ty,tz]
     * @param cur_pq 当前反馈位姿 [x,y,z, w,x,y,z]
     * @param ref_ft 期望力 [fx,fy,fz, tx,ty,tz]
     * @param ref_pq 期望位姿(Position + Quaternion) [x,y,z, w,x,y,z]
     * @param ref_vo 期望速度(Velocity + Omega) [vx,vy,vz, wx,wy,wz]
     * @param ref_ab 期望加速度(Acceleration + Beta) [ax,ay,az, bx,by,bz]
     * @param out_cmd_pq 输出指令位姿 [x,y,z, w,x,y,z]
     * @param out_cmd_vo 【可选】输出指令速度 [vx,vy,vz, wx,wy,wz]
     * @param out_cmd_ab 【可选】输出指令加速度 [ax,ay,az, bx,by,bz]
     */
    auto update(const double* cur_ft, const double* cur_pq, const double* ref_ft, const double* ref_pq,
        const double* ref_vo, const double* ref_ab, double* out_cmd_pq, double* out_cmd_vo = nullptr,
        double* out_cmd_ab = nullptr) -> void ;

private:
    // 阻抗参数
    double M[6];
    double D[6];
    double K[6];

    // 状态积分量
    double pos_offset_[3];      // 累积位置偏差
    double vel_offset_[3];      // 累积速度偏差 (虚拟速度)
    double rot_offset_q_[4];    // 累积姿态偏差 (四元数)
    double ang_vel_offset_[3];  // 累积角速度偏差

    double dt_;       // 控制周期 (s)，用于积分步长
    double lag_tau_;  // 前馈时滞 (s)，默认等于 dt_
};

}  // namespace force
}  // namespace rtb

#endif  // RTB_ADMITTANCE_CONTROLLER_HPP
