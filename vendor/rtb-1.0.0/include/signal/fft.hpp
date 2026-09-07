/**
 * @file fft.hpp
 * @brief 快速傅里叶变换 (FFT) 模块
 * @details 实现了 Cooley-Tukey Radix-2 算法，支持实数输入和复数输出。
 */

#ifndef RTB_SIGNAL_FFT_HPP
#define RTB_SIGNAL_FFT_HPP

#include <complex>
#include <vector>

namespace rtb {
namespace signal {

class FFT {
public:
    // 使用 std::complex 是最通用且标准的选择
    using Complex = std::complex<double>;

    /**
     * @brief 执行前向 FFT (时域 -> 频域)
     * @param input 时域信号 (实数)
     * @return 频域信号 (复数)，大小为 2^N (会自动补零到最近的 2^N)
     */
    static auto forward(const std::vector<double>& input) -> std::vector<Complex> ;

    /**
     * @brief 执行逆向 FFT (频域 -> 时域)
     * @param input 频域信号 (复数)
     * @return 时域信号 (实数，取复数结果的实部并归一化)
     */
    static auto inverse(const std::vector<Complex>& input) -> std::vector<double> ;

    /**
     * @brief 计算幅值谱 (Amplitude Spectrum)
     * @param fft_result FFT计算出的复数结果
     * @return 幅值数组 (|z|)，通常只需要前半部分(Nyquist频率之前)
     */
    static auto computeMagnitude(const std::vector<Complex>& fft_result) -> std::vector<double> ;

    /**
     * @brief 计算频率轴
     * @param n_points FFT点数
     * @param sample_rate 采样率
     * @return 对应的频率数组
     */
    static auto computeFreqAxis(size_t n_points, double sample_rate) -> std::vector<double> ;

private:
    // 核心递归/迭代实现
    static auto fft_core(std::vector<Complex>& a, bool invert) -> void ;
    // 比特反转重排
    static auto bit_reverse(size_t n, size_t bits) -> size_t ;
};

}  // namespace signal
}  // namespace rtb

#endif  // RTB_SIGNAL_FFT_HPP