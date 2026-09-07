/**
 * @file signal_generator.hpp
 * @brief 实时信号生成器 (支持正弦、方波、三角波、锯齿波、Chirp)
 * @details 使用相位累加器原理，避免大时间 t 下的精度丢失，支持动态变频。
 */

#ifndef RTB_SIGNAL_GENERATOR_HPP
#define RTB_SIGNAL_GENERATOR_HPP

#include <random>

namespace rtb {
namespace signal {

enum class WaveType {
    SINE,      // 正弦波
    SQUARE,    // 方波
    TRIANGLE,  // 三角波
    SAWTOOTH   // 锯齿波
};

class SignalGenerator {
public:
    /**
     * @brief 构造函数
     * @param type 波形类型
     * @param frequency_hz 频率 (Hz)
     * @param amplitude 幅值
     * @param phase_deg 初始相位 (度)
     * @param sample_rate 采样率 (Hz)
     */
    SignalGenerator(WaveType type, double frequency_hz, double amplitude, double phase_deg, double sample_rate);

    ~SignalGenerator() = default;

    /**
     * @brief 生成下一个采样点 (实时调用)
     * @return 当前时刻的信号值
     */
    auto step() -> double ;

    /**
     * @brief 重置相位
     */
    auto reset() -> void ;

    // ================= 参数设置 =================
    auto setFrequency(double hz) -> void ;
    auto setAmplitude(double amp) -> void ;
    void setWaveType(WaveType type);
    auto setDutyCycle(double duty) -> void ;

private:
    WaveType type_;
    double frequency_;
    double amplitude_;
    double sample_rate_;
    double duty_cycle_ = 0.5;  // 占空比

    // 核心状态：当前相位 (归一化到 [0, 1.0))
    double current_phase_normalized_;
    double phase_increment_;
};

/**
 * @brief 扫频信号生成器 (Chirp Signal)
 * @details 频率随时间线性增加，常用于系统辨识
 */
class ChirpGenerator {
public:
    ChirpGenerator(double start_freq, double end_freq, double duration, double amplitude, double sample_rate);
    auto step() -> double ;
    auto reset() -> void ;
    auto isFinished() const -> bool ;

private:
    double start_freq_;
    double duration_;
    double amplitude_;
    double dt_;

    double current_time_;
    double k_;  // 频率变化斜率
};

/**
 * @brief 对数扫频信号生成器 (Logarithmic Chirp)
 * @details 频率随时间呈指数增长/衰减 (Octave/sec)。
 * 优化：使用增量乘法避免每步调用 std::pow，适合实时系统。
 */
class LogChirpGenerator {
public:
    LogChirpGenerator(double start_freq, double end_freq, double duration, double amplitude, double sample_rate);
    auto step() -> double ;
    auto reset() -> void ;
    auto isFinished() const -> bool ;

private:
    double start_freq_;
    double end_freq_;
    double duration_;
    double amplitude_;
    double sample_rate_;

    // 运行时状态
    double current_freq_;     // 当前瞬时频率
    double freq_multiplier_;  // 频率每步的增长倍数 (Pre-calculated)
    double current_phase_;    // 当前相位累积
    double current_time_;
};

/**
 * @brief 噪声生成器
 * @details 支持均匀分布、高斯(白噪声)、布朗(红噪声)
 */
enum class NoiseType {
    UNIFORM,   // 均匀分布 [-Amp, Amp]
    GAUSSIAN,  // 高斯白噪声 (Mean=0, StdDev=Amp)
    BROWNIAN   // 布朗噪声 (随机游走，积分白噪声)
};

class NoiseGenerator {
public:
    /**
     * @param type 噪声类型
     * @param amplitude 幅度参数 (对高斯为标准差，对均匀为边界)
     * @param seed 随机种子 (传 0 则使用随机设备生成随机种子)
     */
    NoiseGenerator(NoiseType type, double amplitude, unsigned int seed = 0);

    auto step() -> double ;
    auto setAmplitude(double amp) -> void ;

private:
    NoiseType type_;
    double amplitude_;

    // 随机数引擎
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_uniform_;
    std::normal_distribution<double> dist_normal_;

    // 布朗噪声状态
    double brownian_state_ = 0.0;
};

}  // namespace signal
}  // namespace rtb

#endif  // RTB_SIGNAL_GENERATOR_HPP