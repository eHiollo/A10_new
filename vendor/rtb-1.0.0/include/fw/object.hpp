#ifndef RTB_FRAMEWORK_OBJECT_HPP
#define RTB_FRAMEWORK_OBJECT_HPP

#include <string>
#include <string_view>

namespace rtb::fw {

// 所有 RTB 管理对象的顶层基类
class ObjectRoot {
public:
    explicit ObjectRoot(std::string name) : name_(std::move(name)) {}

    virtual ~ObjectRoot() = default;

    // 禁止赋值
    ObjectRoot& operator=(const ObjectRoot&) = delete;

    virtual auto init() -> void {}

    auto name() const -> const std::string& { return name_; }

protected:
    // 允许子类（如 Skill）调用拷贝构造
    ObjectRoot(const ObjectRoot&) = default;

    std::string name_;
};

// ========================================================
// 标签系统 (Tag System)
// 用法：RTB_TAG(TagName, BaseType) 生成 struct TagName { Type, ID }
// 例：RTB_TAG(MoveSkillTag, MoveSkill) => struct MoveSkillTag
//     TAG_CREATE_AND_LINK(SysHardwareRobot, RobotBase) => struct SysHardwareRobot
// ========================================================
#define RTB_TAG_1(ClassName)                               \
    struct ClassName##Tag {                                \
        using Type = ClassName;                            \
        static constexpr std::string_view ID = #ClassName; \
    };                                                     \
    inline constexpr ClassName##Tag ClassName##Key {}

#define RTB_TAG_2(KeyName, T)                            \
    struct KeyName##Tag {                                \
        using Type = T;                                  \
        static constexpr std::string_view ID = #KeyName; \
    };                                                   \
    inline constexpr KeyName##Tag KeyName##Key {}

#define RTB_TAG_N(_1, _2, N, ...) RTB_TAG_##N
#define TAG_CREATE_AND_LINK(...) RTB_TAG_N(__VA_ARGS__, 2, 1)(__VA_ARGS__)

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_OBJECT_HPP
