#ifndef RTB_FRAMEWORK_SYSTEM_HPP
#define RTB_FRAMEWORK_SYSTEM_HPP

#include <memory>
#include <stdexcept>
#include <unordered_map>

#include "fw/bind.hpp"
#include "fw/lockable.hpp"
#include "fw/object.hpp"
#include "fw/skill.hpp"

namespace rtb::fw {

class System {
public:
    static auto instance() -> System& ;

    /**
     * @brief 清空系统中的所有对象与 get() 缓存
     *
     * 仅适用于尚未 spawn 或所有 task 已析构的场景。若存在持有 MotionBase 锁的
     * task，调用 reset() 会导致未定义行为。用法：sys.reset();
     */
    auto reset() -> void {
        objects_.clear();
        get_cache_().clear();
    }

    // 强制注入 name：始终将 Tag::ID 作为第一个参数传给构造函数
    // 用法：sys.link_tag_to_class<MockRobotTag, MockRobot>();  或
    // link_tag_to_class<Tag, Impl>(cfg, ...);
    /**
     * @brief 将 Tag 与 Impl 实例关联起来
     * 注意：这个函数会创建一个 Impl 实例，并将其与 Tag 关联起来。
     * 因此，如果需要重新创建对象，需要先调用这个函数。
     * 如果 Tag 已经关联了其他 Impl 实例，则会抛出异常。
     * 用法：sys.link_tag_to_class<MockRobotTag, MockRobot>(cfg, ...);  或
     * link_tag_to_class<Tag, Impl>(cfg, ...);
     *
     * @tparam Tag 标签类型
     * @tparam Impl 实现类型
     * @tparam Args 参数类型
     * @param args 传递给 Impl 构造函数的参数
     */
    template <typename Tag, typename Impl, typename... Args>
    auto link_tag_to_class(Args&&... args) -> void {
        static_assert(std::is_base_of<typename Tag::Type, Impl>::value,
            "RTB: Impl must inherit from Tag::Type");

        std::string key(Tag::ID);
        if (objects_.find(key) != objects_.end()) {
            throw std::runtime_error("RTB: Duplicate ID " + key);
        }

        // 核心：无论是否有额外参数，key 始终作为第一个参数注入
        auto ptr = std::make_shared<Impl>(key, std::forward<Args>(args)...);
        ptr->init();
        objects_[key] = std::move(ptr);
    }

    /**
     * @brief 创建一个 Tag 与 Impl 实例关联起来
     *
     * @tparam Tag 标签类型
     * @tparam Args 参数类型
     * @param args 传递给 Impl 构造函数的参数
     */
    template <typename Tag, typename... Args>
    auto create(Args&&... args) -> void {
        link_tag_to_class<Tag, typename Tag::Type>(std::forward<Args>(args)...);
    }

    /**
     * @brief 获取一个 Tag 对应的 Impl 实例
     *
     * 首次调用后缓存指针，热路径 O(1)。reset() 会清空缓存。
     * 用法：auto& motion = sys.get<LeftMotionTag>();
     */
    template <typename Tag>
    auto get() -> typename Tag::Type& {
        std::string key(Tag::ID);
        auto& cache = get_cache_();
        auto it = cache.find(key);
        if (it != cache.end()) {
            return *static_cast<typename Tag::Type*>(it->second);
        }
        auto it2 = objects_.find(key);
        if (it2 == objects_.end()) {
            throw std::runtime_error("RTB: Not Found " + key);
        }
        auto* ptr = dynamic_cast<typename Tag::Type*>(it2->second.get());
        if (!ptr) {
            throw std::runtime_error("RTB: Bad cast for " + key);
        }
        cache[key] = ptr;
        return *ptr;
    }

    /**
     * @brief 获取一个 Tag 对应的 Impl 实例
     *
     * @tparam Tag 标签类型
     * @return Tag::Type& 标签对应的 Impl 实例
     */
    template <typename Tag>
    auto get(const Tag&) -> typename Tag::Type& {
        return get<Tag>();
    }

    // -------------------------------------------------------
    // 辅助：处理单个绑定（查找资源 -> 尝试锁定 -> 注入）
    // -------------------------------------------------------
    template <typename SkillType, typename P, typename R>
    auto process_binding(SkillType* task, PortBinding<P, R>) -> void {
        auto& resource_ref = get<R>();
        auto* resource_ptr = &resource_ref;

        if (!dynamic_cast<typename P::Type*>(resource_ptr)) {
            throw std::runtime_error("RTB: Type mismatch for port " +
                                     std::string(P::ID));
        }

        if (auto* lockable = dynamic_cast<LockableResource*>(resource_ptr)) {
            if (!lockable->try_lock()) {
                throw std::runtime_error("RTB: Resource is BUSY (Locked): " +
                                         std::string(R::ID));
            }
        }

        task->inject_dependency(P::ID, resource_ptr);
    }

    /**
     * @brief 技能孵化接口，只负责克隆与资源绑定 (Wiring)
     * 初始化逻辑由调用方 spawn 后显式调用 task->prepare(cfg)
     *
     * 用法：
     *   auto task = sys.spawn<MultiAxisFollowSkillTag>(
     *       bind<Port::Motion, LeftMotionTag>());
     *   task->prepare(cfg);
     *   while (task->step() == Status::RUNNING) ...
     */
    template <typename SkillTag, typename... Bindings>
    auto spawn(Bindings... bindings) -> std::unique_ptr<typename SkillTag::Type> {
        auto& prototype = get<SkillTag>();
        SkillInterface* cloned_ptr = prototype.clone();
        auto* task = dynamic_cast<typename SkillTag::Type*>(cloned_ptr);
        if (!task) {
            delete cloned_ptr;
            throw std::runtime_error("RTB: Spawn casting failed for " + std::string(SkillTag::ID));
        }
        auto task_ptr = std::unique_ptr<typename SkillTag::Type>(task);
        (void)std::initializer_list<int>{(process_binding(task, bindings), 0)...};
        return task_ptr;
    }

private:
    static auto get_cache_() -> std::unordered_map<std::string, void*>& {
        static std::unordered_map<std::string, void*> cache;
        return cache;
    }

    System() = default;
    std::unordered_map<std::string, std::shared_ptr<ObjectRoot>> objects_;
};

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_SYSTEM_HPP
