#ifndef KEYBOARD_LISTENER_HPP
#define KEYBOARD_LISTENER_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <termios.h>
#endif

namespace rtb {
namespace io {

// --- 线程安全队列 (复刻 Python 的 queue.Queue) ---
template <typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    size_t max_size_;

public:
    explicit ThreadSafeQueue(size_t max_size = 0) : max_size_(max_size) {}

    // 尝试放入数据 (非阻塞, 满了就丢弃或抛出逻辑，复刻 block=False)
    auto put_nowait(const T& item) -> bool {
        std::lock_guard<std::mutex> lock(mutex_);
        if (max_size_ > 0 && queue_.size() >= max_size_) {
            return false;  // Queue Full
        }
        queue_.push(item);
        return true;
    }

    // 尝试获取数据 (非阻塞)
    auto get_nowait(T& item) -> bool {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        item = queue_.front();
        queue_.pop();
        return true;
    }
};

// --- 键盘监听器主类 ---
class KeyboardListener {
public:
    using CallbackType = std::function<void(char)>;
    using QueuePtr = std::shared_ptr<ThreadSafeQueue<bool>>;  // bool 代表 msg

    KeyboardListener();
    ~KeyboardListener();

    auto start() -> void ;
    auto stop() -> void ;
    auto join() -> void ;
    /** 监听线程是否可 join（用于主循环判断是否已退出，如按 ESC） */
    auto joinable() const -> bool ;

    // 事件注册机制
    // event_type: 0 (按下), 1 (松开), 2 (长按/按压与松开)
    auto event_register(char key, int event_type,
        const std::vector<CallbackType>& callbacks,
        const std::vector<QueuePtr>& queues) -> void ;

    auto event_register(const std::vector<char>& keys, int event_type,
        const std::vector<CallbackType>& callbacks,
        const std::vector<QueuePtr>& queues) -> void ;

private:
    struct EventConfig {
        int type;
        std::vector<CallbackType> callbacks;
        std::vector<QueuePtr> queues;
    };

    // 内部状态字典
    std::unordered_map<char, EventConfig> key_events_;
    std::unordered_map<char, bool> status_;
    std::unordered_map<char, std::chrono::steady_clock::time_point> press_time_;
    std::unordered_map<char, std::chrono::steady_clock::time_point> release_time_;
    std::unordered_map<char, std::chrono::steady_clock::time_point> last_seen_time_;

    std::atomic<bool> running_;
    std::thread listener_thread_;

    // POSIX 终端配置缓存 (Ubuntu/macOS 需 termios.h)
    struct termios orig_termios_;

    // 核心逻辑函数
    auto _run() -> void ;
    auto _set_raw_mode() -> void ;
    auto _restore_mode() -> void ;
    auto _on_press(char key) -> void ;
    auto _on_release(char key) -> void ;
    auto _event_handler(char key, bool is_press) -> void ;
    auto _execute_callbacks(char key, int callback_idx) -> void ;
    auto _send_queues(char key, bool msg) -> void ;
};

}  // namespace io
}  // namespace rtb

#endif  // KEYBOARD_LISTENER_HPP