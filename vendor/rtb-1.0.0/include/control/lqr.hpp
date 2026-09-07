/**
 * @file lqr.hpp
 * @brief 通用离散无限时域 LQR（Linear Quadratic Regulator）控制器
 * @details
 * 解决离散线性系统的最优控制问题：
 *   x_{k+1} = A * x_k + B * u_k
 * 最小化二次代价函数：
 *   J = Σ (x_k^T * Q * x_k + u_k^T * R * u_k)
 * 最优控制律：
 *   u_k = -K * x_k
 * 其中 K 通过求解离散代数黎卡提方程（DARE）得到。
 *
 * 特点：
 *   - 支持任意维数：state_dim × control_dim，运行时指定
 *   - 实时安全：init() 一次性分配全部内存，update() 路径零分配
 *   - 通用性：不依赖 Eigen，仅使用 rtb::math 中的矩阵运算
 */
#ifndef RTB_CONTROL_LQR_HPP
#define RTB_CONTROL_LQR_HPP

#include <memory>

namespace rtb {
namespace control {

class LQR {
public:
    // ---- 构造与析构 ----
    LQR();
    ~LQR();

    // ---- Rule of Three/Five（深拷贝语义） ----
    LQR(const LQR& other);
    auto operator=(const LQR& other) -> LQR&;
    LQR(LQR&& other) noexcept;
    auto operator=(LQR&& other) noexcept -> LQR&;

    /**
     * @brief 初始化系统维度并分配内存
     * @param state_dim   状态向量维度 n
     * @param control_dim 控制输入维度 m（m > 0）
     * @details 调用后需依次设置 A, B, Q, R，再调用 computeGain() 求解 K。
     *          所有临时矩阵在此一次性预分配，保证 update() 路径无堆分配。
     */
    auto init(int state_dim, int control_dim) -> void;

    // ---- 参数设置（必须在 init() 之后、computeGain() 之前调用） ----
    auto setA(const double* A) -> void;  // 状态转移矩阵（n × n，行优先）
    auto setB(const double* B) -> void;  // 控制矩阵（n × m，行优先）
    auto setQ(const double* Q) -> void;  // 状态权重矩阵（n × n，行优先，对称半正定）
    auto setR(const double* R) -> void;  // 控制权重矩阵（m × m，行优先，对称正定）

    /**
     * @brief 求解离线黎卡提方程，计算最优反馈增益矩阵 K
     * @param max_iter   最大迭代次数（默认 1000）
     * @param tolerance  收敛阈值（默认 1e-8）
     * @return 是否收敛成功
     * @details 使用迭代法求解 DARE（离散代数黎卡提方程）：
     *            P_{k+1} = Q + A^T * P_k * A - A^T * P_k * B * (R + B^T * P_k * B)^{-1} * B^T * P_k * A
     *          收敛后计算 K = (R + B^T * P * B)^{-1} * B^T * P * A
     *          P 初始化为 Q，通常 100~200 次迭代内收敛。
     */
    auto computeGain(int max_iter = 1000, double tolerance = 1e-8) -> bool;

    /**
     * @brief 计算控制输出 u = -K * x
     * @param x  当前状态向量（长度 n，输入）
     * @param u  控制输出向量（长度 m，输出，由调用方预分配）
     * @details 实时安全：零堆分配，仅使用栈变量与预分配缓冲区。
     */
    auto update(const double* x, double* u) -> void;

    /**
     * @brief 直接获取计算好的反馈增益矩阵 K 的指针
     * @return 指向 K 的常量指针（m × n，行优先）
     * @details 仅提供读取接口，保证矩阵内容不被外部误修改。
     */
    auto gainK() const -> const double*;

    // ---- 维度查询 ----
    auto stateDim() const -> int;
    auto controlDim() const -> int;

    // ---- 状态查询 ----
    auto isInitialized() const -> bool;
    auto isGainReady() const -> bool;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace control
}  // namespace rtb

#endif  // RTB_CONTROL_LQR_HPP