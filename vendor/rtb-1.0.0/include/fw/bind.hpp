#ifndef RTB_FRAMEWORK_BIND_HPP
#define RTB_FRAMEWORK_BIND_HPP

namespace rtb::fw {

/**
 * @brief 绑定描述符：仅用于编译期传递类型信息
 * @tparam P PortTag (Skill 端的端口)
 * @tparam R ResourceTag (System 端具体存在的资源)
 */
template <typename P, typename R>
struct PortBinding {
    using PortTag = P;
    using ResourceTag = R;
};

/**
 * @brief 语法糖：生成 PortBinding
 * @code
 *   rtb::fw::bind<MoveSkill::Port::Actuation, LeftMotionTag>()
 * @endcode
 */
template <typename PortTag, typename ResourceTag>
constexpr auto bind() -> PortBinding<PortTag, ResourceTag> {
    return PortBinding<PortTag, ResourceTag>{};
}

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_BIND_HPP
