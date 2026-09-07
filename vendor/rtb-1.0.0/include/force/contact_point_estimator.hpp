#ifndef RTB_FORCE_CONTACT_POINT_ESTIMATOR_HPP_
#define RTB_FORCE_CONTACT_POINT_ESTIMATOR_HPP_

#include "signal/kalman_filter.hpp"
#include <string>

namespace rtb {
namespace force {

/**
 * @brief 接触点(柔顺中心)在线估计器
 * @details 基于线性卡尔曼滤波，利用 T = r x F 约束，
 * 在线辨识工具末端接触点相对于传感器中心的位移向量 r。
 */
class ContactPointEstimator {
public:
    ContactPointEstimator();
    ~ContactPointEstimator();

    /**
     * @brief 初始化/重置滤波器
     * @param process_noise_q 过程噪声 (建议 1e-6 ~ 1e-8)
     * @param measure_noise_r 测量噪声 (建议 1e-3 ~ 1e-4)
     */
    auto reset(double process_noise_q = 1e-7, double measure_noise_r = 1e-3) -> void ;

    /**
     * @brief 设置最小激振力阈值
     * @param threshold 最小激振力阈值
     */
    auto setMinForceThreshold(double threshold) -> void ;

    /**
     * @brief 核心迭代函数
     * @param ft_input 六维力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]，通常为补偿后的净力/力矩 (Sensor Frame)
     * @return true 如果更新成功 (力足够大), false 如果忽略 (力太小)
     */
    auto update(const double* ft_input) -> bool ;

    /**
     * @brief 获取当前估计的柔顺中心 r
     * @param out_r 返回的 [rx, ry, rz]
     */
    auto getEstimatedPoint(double* out_r) const -> void ;

    /**
     * @brief 判断是否收敛
     * @details 检查协方差矩阵 P 的迹是否小于阈值
     */
    auto isConverged(double threshold = 1e-5) const -> bool ;

    /**
     * @brief 获取当前的不确定度 (P 矩阵的迹)
     */
    auto getUncertainty() const -> double ;

    /**
     * @brief 将估计的柔顺中心 r、不确定度等保存到 JSON 文件
     * @param filename 文件路径，如 rcc_config.json
     * @return 成功返回 true，写入失败返回 false
     */
    auto saveParams(const std::string& filename) const -> bool ;

    /**
     * @brief 从 JSON 文件加载柔顺中心 r、不确定度，并设为 KF 的初始状态
     * @param filename 文件路径，如 rcc_config.json
     * @return 成功返回 true，文件不存在或格式错误返回 false
     */
    auto loadParams(const std::string& filename) -> bool ;

private:
    rtb::signal::KalmanFilter kf_;
    bool initialized_;
    double min_force_threshold_;  // 最小激振力阈值
};

/**
 * @brief 多点接触重心估计器
 * @details 管理多个 ContactPointEstimator，用于辨识多边形工具的几何中心。
 * 初始化时指定点数，运行时通过 ID 切换更新。
 */
class MultiPointContactEstimator {
public:
    /**
     * @brief 构造函数
     * @param num_points 需要辨识的接触点总数 (例如三角形传3)
     */
    explicit MultiPointContactEstimator(int num_points = 3);
    ~MultiPointContactEstimator();

    /**
     * @brief 全局重置
     * @details 重置内部所有的估计器
     */
    auto reset(double process_noise_q = 1e-7, double measure_noise_r = 1e-3) -> void ;

    /**
     * @brief 重新设置接触点数，重新分配内存并重置所有估计器
     * @param num_points 新的接触点总数 (至少为 1，无效时会被钳位为 1)
     * @param process_noise_q 过程噪声 (与 reset 相同，默认 1e-7)
     * @param measure_noise_r 测量噪声 (与 reset 相同，默认 1e-3)
     */
    auto setNumPointsAndReset(int num_points, double process_noise_q = 1e-7, double measure_noise_r = 1e-3) -> void ;

    /**
     * @brief 更新指定点的估计器
     * @param point_id 点的索引 (0 到 num_points-1)
     * @param ft_input 六维力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]，通常为补偿后的净力/力矩
     * @return true 如果该点的估计器更新成功
     */
    auto update(int point_id, const double* ft_input) -> bool ;

    /**
     * @brief 获取当前所有点的几何重心 (平均值)
     * @param out_centroid 返回 [cx, cy, cz]
     */
    auto getCentroid(double* out_centroid) const -> void ;

    /**
     * @brief 获取指定单点的估计结果 (用于调试或绘图)
     */
    auto getPointResult(int point_id, double* out_r) const -> void ;

    /**
     * @brief 获取指定点的不确定度
     */
    auto getPointUncertainty(int point_id) const -> double ;

    /**
     * @brief 检查所有点是否都已收敛
     * @param threshold 收敛阈值
     */
    auto isAllConverged(double threshold = 1e-5) const -> bool ;

    /**
     * @brief 获取点数
     */
    auto getPointCount() const -> int { return static_cast<int>(estimators_.size()); }

private:
    // 使用 vector 存储实体对象
    // 在构造时 resize，之后禁止 push_back，确保无动态内存分配
    std::vector<ContactPointEstimator> estimators_;
};

}  // namespace force
}  // namespace rtb

#endif  // RTB_FORCE_CONTACT_POINT_ESTIMATOR_HPP_