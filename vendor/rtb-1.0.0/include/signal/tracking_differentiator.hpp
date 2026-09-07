#pragma once

#include <memory>
#include <vector>

namespace rtb {
namespace signal {

// ============================================================================
// 1. 底层基础类: 标量一维跟踪微分器 (1st-Order TD)
// ============================================================================
/**
 * @brief 单通道一维跟踪微分器
 * @details 利用韩京清 fhan 函数，追踪输入信号，并提取其平滑的一阶导数
 */
class TD1D {
public:
    TD1D();
    ~TD1D();

    TD1D(const TD1D&) = delete;
    TD1D& operator=(const TD1D&) = delete;
    TD1D(TD1D&&) noexcept;
    TD1D& operator=(TD1D&&) noexcept;

    /**
     * @brief 设置参数
     * @param r  追踪速度因子 (越大追踪越快，但噪声越大)
     * @param h0 滤波因子 (通常大于等于控制周期 dt)
     * @param dt 控制周期
     */
    auto setParams(double r, double h0, double dt) -> void ;

    auto init(double x_init) -> void ;

    /**
     * @brief 更新状态
     * @param v_measure 测量的输入信号
     * @param x1_hat    输出：平滑后的追踪信号
     * @param x2_hat    输出：平滑后的一阶导数
     */
    auto update(double v_measure, double& x1_hat, double& x2_hat) -> void ;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

// ============================================================================
// 2. 级联封装类: 串联跟踪微分器 (Cascaded TD for Accel)
// ============================================================================
/**
 * @brief 级联一维跟踪微分器
 * @details 内部串联两个 TD1D。TD1 提取速度，TD2 追踪速度提取加速度。
 */
class CascadedTD1D {
public:
    CascadedTD1D();
    ~CascadedTD1D();

    CascadedTD1D(const CascadedTD1D&) = delete;
    CascadedTD1D& operator=(const CascadedTD1D&) = delete;
    CascadedTD1D(CascadedTD1D&&) noexcept;
    CascadedTD1D& operator=(CascadedTD1D&&) noexcept;

    /**
     * @brief 分别设置两级 TD 的参数
     * @param r1, h0_1 第一级(位置->速度) 参数
     * @param r2, h0_2 第二级(速度->加速度) 参数。通常 r2 > r1 以减小高阶相位滞后
     */
    auto setParams(double r1, double h0_1, double r2, double h0_2, double dt) -> void ;

    auto init(double x_init) -> void ;

    auto update(double v_measure, double& pos_hat, double& vel_hat, double& acc_hat) -> void ;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

// ============================================================================
// 3. 多轴通用封装类: 面向机器人关节的通用 TD
// ============================================================================
/**
 * @brief 多轴级联跟踪微分器
 * @details 面向 7-DOF 等多轴系统，内部维护 std::vector<CascadedTD1D>，暴露出指针数组接口
 */
class MultiChannelTD {
public:
    MultiChannelTD();
    ~MultiChannelTD();

    MultiChannelTD(const MultiChannelTD&) = delete;
    MultiChannelTD& operator=(const MultiChannelTD&) = delete;
    MultiChannelTD(MultiChannelTD&&) noexcept;
    MultiChannelTD& operator=(MultiChannelTD&&) noexcept;

    auto setInputSize(int input_size) -> void ;
    auto setDt(double dt) -> void ;

    auto setParams(double r1, double h0_1, double r2, double h0_2) -> void ;

    auto init(const double* q_init) -> void ;

    auto update(const double* q_measure, double* q_hat, double* v_hat, double* a_hat) -> void ;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace signal
}  // namespace rtb