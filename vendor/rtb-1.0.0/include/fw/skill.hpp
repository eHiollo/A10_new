#ifndef RTB_FRAMEWORK_SKILL_HPP
#define RTB_FRAMEWORK_SKILL_HPP

#include "fw/lockable.hpp"
#include "fw/object.hpp"
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rtb::fw {

enum class Status { IDLE,
    RUNNING,
    SUCCESS,
    FAILURE };

class SkillInterface : public ObjectRoot {
public:
    using ObjectRoot::ObjectRoot;
    virtual ~SkillInterface() = default;
    virtual auto clone() const -> SkillInterface* = 0;

    virtual auto step() -> Status { return Status::IDLE; }

    virtual auto stop() -> void {}
};

// 模板基类：只负责资源管理 (IoC) 和 Clone，不碰 Config
template <typename Derived>
class SkillBase : public SkillInterface {
public:
    using SkillInterface::SkillInterface;

    ~SkillBase() override {
        for (auto* res : locked_resources_) {
            if (res)
                res->unlock();
        }
    }

    auto clone() const -> SkillInterface* override {
        return new Derived(static_cast<const Derived&>(*this));
    }

    template <typename PortTag>
    auto get() -> typename PortTag::Type* {
        auto it = dependencies_.find(PortTag::ID);
        if (it != dependencies_.end()) {
            return static_cast<typename PortTag::Type*>(it->second);
        }
        throw std::runtime_error("RTB: Port not bound: " +
                                 std::string(PortTag::ID));
    }

    auto inject_dependency(std::string_view port_id, void* resource_ptr) -> void {
        dependencies_[port_id] = resource_ptr;
        auto* obj = static_cast<ObjectRoot*>(resource_ptr);
        if (auto* lockable = dynamic_cast<LockableResource*>(obj)) {
            locked_resources_.push_back(lockable);
        }
    }

private:
    std::unordered_map<std::string_view, void*> dependencies_;
    std::vector<LockableResource*> locked_resources_;
};

#define RTB_SKILL_PORT(SkillName, PortName, ResourceBaseType)            \
    struct PortName {                                                    \
        using Type = ResourceBaseType;                                   \
        static constexpr std::string_view ID = #SkillName "." #PortName; \
    };                                                                   \
    static constexpr PortName PortName##Key {}

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_SKILL_HPP
