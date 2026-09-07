/**
 * @file force_sensor_calibrator.hpp
 * @brief 六维力传感器重力补偿与标定模块 (重构版 - 使用统一数学库)
 * @details 算法参考: 基于六维力传感器的工业机器人末端负载受力感知研究 (张立建 等)
 * @note 实时版本：使用固定大小数组，避免动态内存分配
 */

#ifndef RTB_FORCE_SENSOR_CALIBRATOR_HPP
#define RTB_FORCE_SENSOR_CALIBRATOR_HPP

#include <string>

namespace rtb {
namespace force {

// 最大采样点数量（预分配内存）
constexpr int MAX_CALIBRATION_SAMPLES = 100;

// 标定参数结构体
struct CalibParams {
    double mass{0.0};      // 负载质量 (kg) (由 G_world 模长反推)
    double G_world[3]{0};  // 世界坐标系下的重力矢量 [Gx, Gy, Gz] (N)
    double F_bias[3]{0};   // 力零点偏置 [Fx0, Fy0, Fz0] (N)

    double cog[3]{0};     // 重心坐标 [Cx, Cy, Cz] (m) (相对于传感器坐标系)
    double T_bias[3]{0};  // 力矩零点偏置 [Tx0, Ty0, Tz0] (Nm)

    bool is_calibrated{false};
};

// 标定采样点数据（用于分析和导出）
struct CalibSampleRecord {
    double pose[7];    // 传感器位姿 [x, y, z, qw, qx, qy, qz]
    double wrench[6];  // 去零偏后的力数据 [Fx, Fy, Fz, Tx, Ty, Tz]
};

class ForceSensorCalibrator {
public:
    ForceSensorCalibrator();
    ~ForceSensorCalibrator();

    /**
     * @brief 设置传感器相对于法兰的固定变换
     * @param T_sensor_to_flange 4x4齐次变换矩阵 (默认为单位矩阵)
     * @details 真机使用时，如果力传感器坐标系与法兰不重合，需要设置此变换
     *          例如：传感器相对法兰沿Z平移0.05m，绕Z旋转45度
     */
    auto setFlangeToSensorTransform(const double* T_sensor_to_flange) -> void ;

    /**
     * @brief 重置采集数据
     */
    auto resetCollection() -> void ;

    /**
     * @brief 添加一组标定采样点 (在静态下采集)
     * @param pm_flange 机器人末端法兰的 4x4 齐次变换矩阵 (单位: m)
     * @param raw_wrench 传感器原始读数 [Fx, Fy, Fz, Tx, Ty, Tz] (单位: N, Nm)
     * @return 成功返回 true，超过最大样本数返回 false
     * @details 函数内部会自动：
     *          1. 减去零偏值
     *          2. 计算传感器坐标系位姿 = pm_flange * T_sensor_to_flange
     */
    auto addSample(const double* pm_flange, const double* raw_wrench) -> bool ;

    /**
     * @brief 获取当前采样点数量
     */
    auto getSampleCount() const -> int ;

    /**
     * @brief 获取采样点记录（用于分析）
     * @param index 采样点索引 (0 <= index < getSampleCount())
     * @param record 输出的记录数据
     * @return 成功返回 true，索引越界返回 false
     */
    auto getSampleRecord(int index, CalibSampleRecord& record) const -> bool ;

    /**
     * @brief 导出采样点数据到CSV
     * @param filename 文件名
     */
    auto exportSamplesToCSV(const std::string& filename) const -> void ;

    /**
     * @brief 从CSV文件读取采样点数据
     * @param filename CSV文件名
     * @return 成功返回 true，失败返回 false
     * @details CSV格式: x,y,z,qw,qx,qy,qz,Fx,Fy,Fz,Tx,Ty,Tz
     *          读取的数据会加载到内部采样缓冲区；不会清空已通过 loadParams 加载的标定参数。
     */
    auto loadSamplesFromCSV(const std::string& filename) -> bool ;

    /**
     * @brief 离线标定主流程（用于算法测试）
     * @param csv_file CSV数据文件路径（由 exportSamplesToCSV 保存）
     * @param init_json_file 初始JSON配置文件路径（包含初始的 T_flange_to_sensor 等参数）
     * @param output_json_file 输出JSON配置文件路径（保存标定结果）
     * @return 成功返回 true，失败返回 false
     * @details 执行完整的离线标定流程：
     *          1. calibrateInstallation() - 定基座重力向量 G
     *          2. calibrateSensorRotation() - 定传感器旋转 R
     *          3. computeCalibration() - 定负载 Mass/CoG
     *          标定结果会自动保存到 output_json_file
     */
    auto offlineCalibration(
        const std::string& csv_file, const std::string& init_json_file, const std::string& output_json_file) -> bool ;

    /**
     * @brief 执行标定计算 (最小二乘法)
     * @return 成功返回 true，样本不足或计算失败返回 false
     */
    auto computeCalibration() -> bool ;

    /**
     * @brief 自动校准安装倾角（基座不平补偿）- 迭代计算真实重力向量
     * @return 成功返回 true，失败返回 false
     * @details 通过迭代法计算真实的重力向量，解决基座不平问题
     *          需要在 computeCalibration() 之后调用
     */
    auto calibrateInstallation() -> bool ;

    /**
     * @brief 自动校准传感器旋转安装误差（SVD 对齐）
     * @return 成功返回 true，失败返回 false
     * @details 使用 SVD 分解计算最佳旋转矩阵，修正传感器安装角度误差
     *          需要在 computeCalibration() 之后调用
     */
    auto calibrateSensorRotation() -> bool ;

    /**
     * @brief 计算标定后的均方根误差 (RMSE)
     * @return std::pair<double, double> pair.first=力误差(N), pair.second=力矩误差(Nm)
     * @details Force RMSE (N)：表示在把重力扣除后，你的力传感器读数和“理论零值”平均差了多少。
     *          < 0.5 N：对于大多数 50kg 量程的传感器，这是极高精度的表现。
     *          0.5 ~ 1.0 N：工业现场可以接受的范围（考虑到线缆拉扯）。
     *          > 2.0 N：肯定有问题。要么是碰到东西了，要么是标定时机器人没停稳。
     *          Torque RMSE (Nm)：通常应该在 0.01 ~ 0.1 Nm 之间。如果这个值很大，通常意味着重心 (CoG)
     * 没算准，或者法兰面不平。
     */
    auto computeRMSE() -> std::pair<double, double> ;

    /**
     * @brief 获取标定后的参数
     */
    auto getParams() const -> CalibParams ;

    /**
     * @brief 设置已知参数 (用于从文件加载)
     */
    auto setParams(const CalibParams& params) -> void ;

    /**
     * @brief 获取质量
     */
    auto getMass() const -> double ;

    /**
     * @brief 设置质量
     */
    auto setMass(double mass) -> void ;

    /**
     * @brief 获取重力矢量 (世界坐标系)
     * @param out_gravity 输出的重力矢量 [Gx, Gy, Gz] (N)
     */
    auto getGravity(double* out_gravity) const -> void ;

    /**
     * @brief 设置重力矢量 (世界坐标系)
     * @param gravity 重力矢量 [Gx, Gy, Gz] (N)
     */
    auto setGravity(const double* gravity) -> void ;

    /**
     * @brief 获取重心坐标
     * @param out_cog 输出的重心坐标 [Cx, Cy, Cz] (m)
     */
    auto getCoG(double* out_cog) const -> void ;

    /**
     * @brief 设置重心坐标
     * @param cog 重心坐标 [Cx, Cy, Cz] (m)
     */
    auto setCoG(const double* cog) -> void ;

    /**
     * @brief 获取力偏置
     * @param out_bias 输出的力偏置 [Fx0, Fy0, Fz0] (N)
     */
    auto getForceBias(double* out_bias) const -> void ;

    /**
     * @brief 设置力偏置
     * @param bias 力偏置 [Fx0, Fy0, Fz0] (N)
     */
    auto setForceBias(const double* bias) -> void ;

    /**
     * @brief 获取力矩偏置
     * @param out_bias 输出的力矩偏置 [Tx0, Ty0, Tz0] (Nm)
     */
    auto getTorqueBias(double* out_bias) const -> void ;

    /**
     * @brief 设置力矩偏置
     * @param bias 力矩偏置 [Tx0, Ty0, Tz0] (Nm)
     */
    auto setTorqueBias(const double* bias) -> void ;

    /**
     * @brief 获取传感器到法兰的变换矩阵
     * @param out_transform 输出的 4x4 变换矩阵
     */
    auto getFlangeToSensorTransform(double* out_transform) const -> void ;

    /**
     * @brief 保存参数到文件
     * @return 成功返回 true，写入失败返回 false
     */
    auto saveParams(const std::string& filename) -> bool ;

    /**
     * @brief 从文件加载参数
     * @return 成功返回 true，文件不存在或格式错误返回 false
     */
    auto loadParams(const std::string& filename) -> bool ;

    /**
     * @brief 设置力死区 (用于消除小力抖动)
     * @param deadzone 6维死区 [Fx, Fy, Fz, Tx, Ty, Tz]，负值表示不启用
     * @details 补偿后的力如果小于死区，则输出为0
     */
    auto setDeadzone(const double* deadzone) -> void ;

    /**
     * @brief 设置输出饱和限制 (防止异常大力)
     * @param saturation 6维饱和限制 [Fx, Fy, Fz, Tx, Ty, Tz]，负值表示不限制
     * @details 补偿后的力如果超过饱和值，则限幅到饱和值
     */
    auto setSaturation(const double* saturation) -> void ;

    /**
     * @brief 获取力死区设置
     * @param out_deadzone 输出的死区 [Fx, Fy, Fz, Tx, Ty, Tz]，负值表示未启用
     */
    auto getDeadzone(double* out_deadzone) const -> void ;

    /**
     * @brief 获取输出饱和限制设置
     * @param out_saturation 输出的饱和限制 [Fx, Fy, Fz, Tx, Ty, Tz]，负值表示未限制
     */
    auto getSaturation(double* out_saturation) const -> void ;

    /**
     * @brief 检查是否已标定
     */
    auto isCalibrated() const -> bool ;

    /**
     * @brief 设置标定状态
     */
    auto setCalibrated(bool calibrated) -> void ;

    /**
     * @brief 设置柔顺中心 RCC 相对传感器原点的位置（传感器坐标系，单位 m）
     * @param r_xyz [px, py, pz]，与 ContactPointEstimator 保存的 r 一致
     */
    auto setRccVector(const double* r_xyz) -> void ;

    /**
     * @brief 获取柔顺中心 RCC 相对传感器原点的位置（传感器坐标系，单位 m）
     * @param out_r_xyz 输出 [px, py, pz]
     */
    auto getRccVector(double* out_r_xyz) const -> void ;

    /**
     * @brief 获取补偿后的纯外力 (Real-time)
     * @param current_pm_flange 当前末端法兰位姿 4x4
     * @param raw_wrench 当前传感器原始读数 6D
     * @param out_comp_wrench 输出: 补偿后的力 [Fx, Fy, Fz, Tx, Ty, Tz]
     * @param output_frame 输出坐标系：0=传感器坐标系, 1=基坐标系(默认), 2=基坐标系且力矩关于 RCC
     * @details 函数内部会自动减去零偏，使用传感器坐标系位姿进行补偿，
     *          并应用死区和饱和限制
     *
     * 【重要】输出的力矩参考点：
     *   - output_frame=0: 力矩关于传感器中心（最直观）
     *   - output_frame=1: 力矩关于传感器中心（用基坐标系表示）
     *   - output_frame=2: 基坐标系表示，力矩关于 RCC；RCC 由 setRccVector 给出（传感器系偏移），
     *     内部等价于先得到 frame=1 再按 rtb::math::change_wrench_point 将参考点从传感器中心平移到 RCC
     */
    auto getCompensatedWrench(
        const double* current_pm_flange, const double* raw_wrench, double* out_comp_wrench, int output_frame = 1) -> void ;

    /**
     * @brief 快速去皮 (Tare)：基于已知负载参数，计算当前的纯零偏
     * @param pm 当前末端位姿矩阵 (4x4)
     * @param current_wrench 当前传感器测量的平均力/力矩 (6x1)
     * @param filename 力传感器的配置文件路径（更新零偏值）
     * @return true 成功, false 失败(未标定或参数异常)
     */
    auto tare(const double* pm, const double* current_wrench, const std::string& filename) -> bool ;

private:
    // 内部数据结构，用于存储采样点（固定大小数组）
    struct SampleData {
        double pm[16];     // 传感器坐标系位姿
        double wrench[6];  // 去零偏后的力数据
    };

    // 固定大小的数据缓冲区（实时性保证）
    SampleData samples_[MAX_CALIBRATION_SAMPLES];
    CalibSampleRecord sample_records_[MAX_CALIBRATION_SAMPLES];
    int sample_count_;  // 当前采样点数量

    CalibParams params_;

    double T_flange_to_sensor_[16];  // 法兰到传感器的固定变换
    double rcc_xyz_[3];              // RCC 相对传感器原点（传感器坐标系，m）
    double deadzone_[6];             // 力死区，负值表示不启用
    double saturation_[6];           // 力饱和限制，负值表示不限制

    /**
     * @brief 使用 SVD 计算两个向量集合的最佳旋转矩阵 (Kabsch 算法)
     * @param src_vectors 源向量列表（每个向量是3元素数组）
     * @param dst_vectors 目标向量列表（每个向量是3元素数组）
     * @param count 向量数量
     * @param R_out 输出的3x3旋转矩阵
     */
    auto solveRotation(const double (*src_vectors)[3], const double (*dst_vectors)[3], int count, double* R_out) -> void ;
};

}  // namespace force
}  // namespace rtb

#endif  // RTB_FORCE_SENSOR_CALIBRATOR_HPP
