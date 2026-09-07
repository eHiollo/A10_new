#ifndef RTB_PLAN_INPUT_INTERPOLATOR_H_
#define RTB_PLAN_INPUT_INTERPOLATOR_H_

#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>

namespace rtb {
namespace plan {

/// 输入插值器：基于前瞻池的轨迹插值
class InputInterpolator {
public:
    using InputGenerator = std::function<std::int64_t(double* input)>;

    /**
     * @brief 获取当前插值池中可用的插值点数量。
     * @return 插值点数量。
     */
    auto interpolationSize() const -> size_t;
    /**
     * @brief 设置外部输入生成器回调。
     * @param input_generator 用于生成原始输入点的回调函数。
     */
    auto setInputGenerator(const InputGenerator& input_generator) -> void;

    /**
     * @brief 设置输入向量维度。
     * @param input_size 输入维度大小。
     */
    auto setInputSize(size_t input_size) -> void;
    /**
     * @brief 获取输入向量维度。
     * @return 当前输入维度。
     */
    auto inputSize() const -> size_t;

    /**
     * @brief 设置离散时间步长。
     * @param dt 时间步长（秒）。
     */
    auto setDt(double dt) -> void;
    /**
     * @brief 获取离散时间步长。
     * @return 当前时间步长（秒）。
     */
    auto dt() const -> double;

    /**
     * @brief 获取前瞻池容量。
     * @return 前瞻池可容纳的点数。
     */
    auto poolSize() const -> size_t;
    /**
     * @brief 设置前瞻池容量。
     * @param size 前瞻池容量。
     */
    auto setPoolSize(size_t size) -> void;

    /**
     * @brief 按当前参数分配内部缓存内存。
     */
    auto allocateMemory() -> void;
    /**
     * @brief 使用初始输入位置初始化插值器状态。
     * @param init_input_pos 初始输入位置数组指针。
     */
    auto init(const double* init_input_pos) -> void;

    /**
     * @brief 设置位置上限数组指针（内存由外部持有）。
     * @param max_poss 各维位置上限数组指针。
     */
    auto setMaxPosPtr(const double* max_poss) -> void;
    /**
     * @brief 设置位置下限数组指针（内存由外部持有）。
     * @param min_poss 各维位置下限数组指针。
     */
    auto setMinPosPtr(const double* min_poss) -> void;

    /**
     * @brief 触发一次输入生成并更新插值池。
     * @return 新插入节点的返回码（即本次调用 `InputGenerator` 的返回值），语义如下：
     *         - `0`：常用于“正常/无特殊状态”的节点（初始化节点默认也是 `0`）；
     *         - `>0`：由上层规划器定义的运行期状态码或节点编号（程序运行时确定）；
     *         - `<0`：由上层规划器定义的异常码；此时内部仍会插入节点，但位置退回上一节点以保持轨迹连续。
     */
    auto generateInput() -> std::int64_t;

    /**
     * @brief 获取参数 s 对应的插值输入。
     * @param s 归一化插值参数。
     * @param p 输出缓冲区指针。
     * @return 查询返回码。
     */
    auto getInputAt(double s, double* p) -> std::int64_t;

    /**
     * @brief 获取参数 s 对应段的返回码。
     * @param s 归一化插值参数。
     * @return 与 `s` 所在离散采样段对应的节点返回码，语义如下：
     *         - 固定值：`0` 表示初始化段/无特殊状态；
     *         - 运行期值：`>0` 表示该离散时刻的有效节点状态（如阶段号/节点号，由上层定义）；
     *         - 运行期值：`<0` 表示该离散时刻的异常状态（由上层定义，位置按回退策略保持连续）；
     *         - 当 `s` 超过当前已生成范围时，返回“最后一个已生成节点”的返回码。
     */
    auto retCodeAt(double s) -> std::int64_t;
    /**
     * @brief 获取最终段的返回码。
     * @return 当前最后一个已生成节点的返回码，取值语义与 `retCodeAt()` 一致：
     *         - 固定值：`0` 表示仅完成初始化或无特殊状态；
     *         - 运行期值：`>0` 表示最新离散时刻的有效节点状态；
     *         - 运行期值：`<0` 表示最新离散时刻的异常状态（位置已按回退策略处理）。
     */
    auto finalRetCode() const -> std::int64_t;
    /**
     * @brief 获取当前缓存中的最终参数值。
     * @return 最终参数 s。
     */
    auto finalS() const -> double;

    InputInterpolator();
    ~InputInterpolator();
    InputInterpolator(const InputInterpolator&) = delete;
    InputInterpolator& operator=(const InputInterpolator&) = delete;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace plan

}  // namespace rtb

#endif
