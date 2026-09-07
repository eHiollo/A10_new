#ifndef RTB_PLAN_INPUT_SMOOTHER_H_
#define RTB_PLAN_INPUT_SMOOTHER_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace rtb {
namespace plan {

/// 输入平滑器：在插值轨迹上做速度/加速度约束平滑
class InputSmoother {
public:
    using InputGenerator = std::function<std::int64_t(double* input)>;

    auto setInputGenerator(const InputGenerator& input_generator) -> void;
    auto setInputSize(size_t input_size) -> void;
    auto inputSize() const -> size_t;
    auto setLookAheadCount(size_t count) -> void;
    auto lookAheadCount() const -> size_t;
    auto setDt(double dt) -> void;
    auto dt() const -> double;

    auto setMaxPos(const std::vector<double>& pos) -> void;
    auto setMaxVel(const std::vector<double>& vel) -> void;
    auto setMaxAcc(const std::vector<double>& acc) -> void;
    auto setMinPos(const std::vector<double>& pos) -> void;
    auto setMinVel(const std::vector<double>& vel) -> void;
    auto setMinAcc(const std::vector<double>& acc) -> void;
    auto maxPos() const -> const std::vector<double>&;
    auto maxVel() const -> const std::vector<double>&;
    auto maxAcc() const -> const std::vector<double>&;
    auto minPos() const -> const std::vector<double>&;
    auto minVel() const -> const std::vector<double>&;
    auto minAcc() const -> const std::vector<double>&;

    auto allocateMemory() -> void;
    auto init(const double* init_input_pos) -> void;

    /**
     * @brief 动态重规划初始化
     * @param q 机械臂当前真实/指令位置
     * @param v 机械臂当前真实/指令速度 (可传 nullptr 视为 0)
     * @param a 机械臂当前真实/指令加速度 (可传 nullptr 视为 0)
     */
    auto reInit(const double* q, const double* v = nullptr, const double* a = nullptr) -> void ;

    /**
     * @brief 计算下一拍平滑后的输入并写入 p。
     *
     * @param p 下一拍平滑后的输入向量，长度为 inputSize()
     * @return std::int64_t 返回码，语义如下：
     *         - `0`：常用于“正常/无特殊状态”的节点（初始化节点默认也是 `0`）；
     *         - `>0`：由上层规划器定义的运行期状态码或节点编号（程序运行时确定）；
     *         - `<0`：由上层规划器定义的异常码；此时内部仍会插入节点，但位置退回上一节点以保持轨迹连续。
     */
    auto getNextInput(double* p) -> std::int64_t;

    /**
     * @brief 当前轨迹参数 s（上一拍 getNextInput 输出对应的弧长/参数），用于验证“路径不变、仅时间缩放”
     * @return double 当前轨迹参数 s
     */
    auto getCurrentS() const -> double;

    /**
     * @brief 当前 s 对时间的变化率 ṡ（即 ds0_ = (s2_-s1_)/dt），与 getCurrentS() 对应同一拍
     * @return double 当前 s 对时间的变化率 ṡ
     */
    auto getCurrentSDot() const -> double;

    InputSmoother();
    ~InputSmoother();
    InputSmoother(const InputSmoother&) = delete;
    InputSmoother& operator=(const InputSmoother&) = delete;

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace plan
}  // namespace rtb

#endif
