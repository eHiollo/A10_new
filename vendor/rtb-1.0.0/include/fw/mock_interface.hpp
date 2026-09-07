#ifndef RTB_FRAMEWORK_MOCK_INTERFACE_HPP
#define RTB_FRAMEWORK_MOCK_INTERFACE_HPP

#include "fw/motion.hpp"
#include <array>

namespace rtb::fw::mock {

/**
 * @brief 模拟运动学，关节状态存于内存，用于离线开发与测试
 */
class MockMotion : public MotionBase {
public:
    static constexpr int DEFAULT_JOINT_N = 7;

    MockMotion() : MotionBase("MockMotion") {}

    using MotionBase::MotionBase;

    auto jointNum() const -> int override { return DEFAULT_JOINT_N; }

    auto getJoints(double* q) const -> bool override {
        for (int i = 0; i < DEFAULT_JOINT_N && i < 32; ++i)
            q[i] = joints_[i];
        return true;
    }

    auto setJoints(const double* q) -> bool override {
        for (int i = 0; i < DEFAULT_JOINT_N && i < 32; ++i)
            joints_[i] = q[i];
        return true;
    }

private:
    std::array<double, 32> joints_{};
};

}  // namespace rtb::fw::mock

#endif  // RTB_FRAMEWORK_MOCK_INTERFACE_HPP
