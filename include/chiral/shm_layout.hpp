#pragma once

#include "chrial.hpp"
#include <atomic>
#include <cstdint>

namespace talos::chiral::ipc {

// ============ 三重缓冲状态编码 ============

namespace detail {
constexpr uint8_t FLAG_NEW   = 0x80; // Bit 7: 有新数据
constexpr uint8_t INDEX_MASK = 0x03; // Bits 0-1: 槽位索引 (0-2)
} // namespace detail

// ============ 三重缓冲布局 ============

/**
 * @brief 三重缓冲共享内存布局
 *
 * 3个槽位循环使用：
 * - write_idx: 生产者当前写入的槽位
 * - read_idx: 消费者当前读取的槽位
 * - ready_idx: state 中编码的就绪槽位
 *
 * 状态编码 (uint8_t atomic):
 *   Bit 7: FLAG_NEW - 有新数据可读
 *   Bits 0-1: ready_index - 就绪槽位索引 (0-2)
 */
template <typename T>
struct TripleBufferLayout {
    std::atomic<uint8_t> state{0}; // 状态字
    uint8_t write_idx{0};          // 生产者写入槽位
    uint8_t read_idx{0};           // 消费者读取槽位
    uint8_t _pad[5];               // 填充到 12 字节

    // 槽位序列锁：偶数稳定，奇数写入中
    std::atomic<uint64_t> slot_seq[3]{{0}, {0}, {0}};

    T slots[3]; // 3个数据槽位
};

static_assert(sizeof(std::atomic<uint8_t>) == 1, "atomic<uint8_t> must be 1 byte");

// ============ TalosData 三重缓冲 ============

struct TalosTripleBuffer {
    std::atomic<uint8_t> state{0};
    uint8_t write_idx{0};
    uint8_t read_idx{0};
    uint8_t _pad[5];
    std::atomic<uint64_t> slot_seq[3]{{0}, {0}, {0}};

    chrial::TalosData slots[3]; // 自然对齐
};

// 验证大小
static_assert(sizeof(TalosTripleBuffer) >= sizeof(chrial::TalosData) * 3 + 32);

} // namespace talos::chiral::ipc
