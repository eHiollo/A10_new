#ifndef RTB_UTILS_HPP
#define RTB_UTILS_HPP

#include <atomic>
#include <cstring>
#include <ctime>
#include <new>          // for std::hardware_destructive_interference_size
#include <type_traits>  // for is_trivially_copyable

namespace rtb {
namespace core {

// 自动获取当前CPU架构的 Cache Line 大小，防止伪共享
#ifdef __cpp_lib_hardware_interference_size
using std::hardware_destructive_interference_size;
#else
// 64 bytes 是 x86/ARM 最常见的缓存行大小
constexpr std::size_t hardware_destructive_interference_size = 64;
#endif

// ============================================================================
// 多线程异步读写工具
// ============================================================================

template <typename T>
class SmartSeqLock {
    // 【安全检查】必须确保 T 是可以被 memcpy 的
    static_assert(std::is_trivially_copyable<T>::value, "SmartSeqLock T must be trivially copyable (POD types only)!");

public:
    SmartSeqLock() { seq_.store(0); }

    auto write(const T& data) -> void {
        // 1. 版本号变奇数 (seq + 1)，表示“正在写”
        // 使用 memory_order_relaxed 即可，因为后续的 fence 会保证顺序
        uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);

        // 确保 seq 更新对其他线程可见后，才开始写数据
        std::atomic_thread_fence(std::memory_order_release);

        // 2. 拷贝数据
        data_ = data;

        // 确保数据写完后，才更新 seq
        std::atomic_thread_fence(std::memory_order_release);

        // 3. 版本号变偶数 (seq + 2)，表示“写完了”
        seq_.store(s + 2, std::memory_order_release);
    }

    class Reader {
    public:
        // 构造函数参数用引用 (T&)，更符合 C++ 习惯，保证调用时传入的对象一定存在
        explicit Reader(SmartSeqLock<T>& lock) : lock_(&lock), last_seq_(0) {
            // 在初始化列表中，取引用的地址 (&lock) 存给指针成员
            // 由于 lock_ 是指针，编译器会自动生成默认的拷贝构造和赋值函数
            // Reader r2 = r1;  // OK
            // r2 = r1;         // OK (只是复制了指针地址，这就对了，因为它们读同一个锁)
        }

        // 返回 true 表示有新数据且读取成功
        // 返回 false 表示无新数据，或读取过程中发生了冲突（此时保持 val 不变）
        auto try_load(T& val) -> bool {
            // 1. Peek Check: 极其高效的检查
            // 如果 seq 是奇数（正在写），或者 seq 没变（没新数据），直接返回
            uint32_t s1 = lock_->seq_.load(std::memory_order_acquire);

            if ((s1 & 1) != 0) {
                // 正在写，发生冲突，放弃读取（下个周期再来）
                return false;
            }
            if (s1 == last_seq_) {
                // 是偶数，但和上次读的一样，说明没有新数据
                return false;
            }

            // 2. 乐观读取 (Optimistic Read)
            // 先把数据考出来，不管是不是坏的
            T temp_data = lock_->data_;

            // 内存屏障：确保数据读取在检查 s2 之前完成
            std::atomic_thread_fence(std::memory_order_acquire);

            // 3. Consistency Check
            uint32_t s2 = lock_->seq_.load(std::memory_order_relaxed);

            // 如果 s1 == s2，说明读取过程中 seq 没变过，且 s1 是偶数
            if (s1 == s2) {
                val = temp_data;  // 数据有效，赋值给用户
                last_seq_ = s1;   // 更新 reader 状态
                return true;
            }

            // 读的过程中发生了写入，数据可能损坏，返回失败，等待下一次循环重读
            return false;
        }

        // 强制重置状态（比如机器人重启时用）
        auto reset() -> void { last_seq_ = 0; }

    private:
        SmartSeqLock<T>* lock_;  // 内部成员用指针 (T*)，为了支持 operator=
        uint32_t last_seq_;
    };

private:
    // 强制按缓存行对齐，彻底解决 False Sharing
    alignas(hardware_destructive_interference_size) std::atomic<uint32_t> seq_;
    alignas(hardware_destructive_interference_size) T data_;

    // 允许 Reader 访问私有成员
    friend class Reader;
};

// 适用情景：SWMR (Single Writer, Multiple Readers)
// 特殊情况：若有多写入者，则需要在写入前争抢到 mutex
// 相对于std::mutex的优势
//      开销：用户态 vs. 内核态。几十纳秒 (ns)。非常稳定
//      Writer(写入者) 的行为：
//      - Seqlock 的 Writer：永远不睡觉，永远不求助操作系统。写完就走。
//      - Mutex 的 Writer：如果 Writer 拿锁的时候，刚好 Reader 还没解锁（虽然几率小），Writer
//        可能会被迫进入内核排队。这会导致 Writer 所在的线程（比如 NRT 通讯线程）出现不可预测的延迟。
//
// ============= 使用方法 =============
// 定义数据结构
// struct PoseData {
//     double arr[7];  // x, y, z, qw, qx, qy, qz
// };
//
// // 创建一个全局的 SmartSeqLock 类的实例
// SmartSeqLock<PoseData> g_target_pose;
//
// ==========================================
// 写入者线程 (模拟 30Hz 遥操作信号输入)
// ==========================================
// void writer_thread_func() {
//     double t = 0.0;
//
//     while (true) {
//         // 1. 准备数据 (模拟生成一个绕圆运动的轨迹)
//         PoseData new_pose;
//         new_pose.arr[0] = 0.5 + 0.1 * std::cos(t);  // x
//         new_pose.arr[1] = 0.1 * std::sin(t);        // y
//         new_pose.arr[2] = 0.3;                      // z
//         new_pose.arr[3] = 1.0;
//         new_pose.arr[4] = 0;  // qw, qx
//         new_pose.arr[5] = 0;
//         new_pose.arr[6] = 0;  // qy, qz
//
//         // 2. 【核心调用】无锁写入，永远不阻塞
//         // 即使此刻 RT 线程正在读，这里也瞬间完成
//         g_target_pose.write(new_pose);
//
//         // 模拟 30Hz 频率
//         std::this_thread::sleep_for(std::chrono::milliseconds(33));
//     }
// }
//
// void real_time_loop() {
//     static PoseData current_pose = {0};
//
//     // 【定义一个静态 Reader】
//     // 它内部自动维护了 last_seq，而且只属于这个函数
//     static SmartSeqLock<PoseData>::Reader reader{g_target_pose};
//
//     // 【调用极简】
//     if (reader.try_load(current_pose)) {
//         // 读到了新数据
//     }
//
//     robot.move(current_pose.arr);
// }
//
// void logger_thread() {
//     PoseData log_pose;
//     // 这里定义另一个 Reader，它有自己独立的 last_seq
//     SmartSeqLock<PoseData>::Reader log_reader{g_target_pose};
//
//     while (true) {
//         if (log_reader.try_load(log_pose)) {
//             save_to_file(log_pose);
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
// }

}  // namespace core
}  // namespace rtb

#endif
