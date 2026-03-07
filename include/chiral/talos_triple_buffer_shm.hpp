#pragma once

#include "chrial.hpp"
#include "shm_layout.hpp"
#include "shm_triple_buffer.hpp"
#include <atomic>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>

namespace talos::chiral::ipc {

// ============ 错误类型 ============

enum class ShmError : uint8_t {
    OpenFailed      = 0,
    TruncateFailed  = 1,
    MapFailed       = 2,
    NotFound        = 3,
    InvalidMagic    = 4,
    VersionMismatch = 5,
};

// ============ 常量定义 ============

inline constexpr uint32_t TALOS_SHM_MAGIC   = 0x544C4454; // "TLDT"
inline constexpr uint32_t TALOS_SHM_VERSION = 2;          // v2: 三重缓冲
inline constexpr const char* TALOS_SHM_NAME = "/talos_data_triple";

// ============ 共享内存头部 ============

struct alignas(64) ShmHeader {
    uint32_t magic;
    uint32_t version;
    uint8_t _pad[56];
};
static_assert(sizeof(ShmHeader) == 64);

// ============ 完整共享内存布局 ============

struct TalosShmTriple {
    ShmHeader header;
    TalosTripleBuffer buffer;
};

inline constexpr size_t TALOS_SHM_SIZE = sizeof(TalosShmTriple);

// ============ RAII 共享内存区域 ============

class ShmRegion {
public:
    ~ShmRegion() noexcept {
        if (owner_ && !path_.empty()) {
            (void)::shm_unlink(path_.c_str());
        }

        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, size_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    ShmRegion(ShmRegion&& other) noexcept
        : data_(std::exchange(other.data_, nullptr))
        , size_(std::exchange(other.size_, 0))
        , fd_(std::exchange(other.fd_, -1))
        , owner_(std::exchange(other.owner_, false))
        , path_(std::move(other.path_)) {}

    ShmRegion& operator=(ShmRegion&& other) noexcept {
        if (this != &other) {
            cleanup();
            data_  = std::exchange(other.data_, nullptr);
            size_  = std::exchange(other.size_, 0);
            fd_    = std::exchange(other.fd_, -1);
            owner_ = std::exchange(other.owner_, false);
            path_  = std::move(other.path_);
        }
        return *this;
    }

    ShmRegion(const ShmRegion&)            = delete;
    ShmRegion& operator=(const ShmRegion&) = delete;

    [[nodiscard]] static std::expected<ShmRegion, ShmError> create(const char* name, size_t size) {
        const int fd = ::shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0644);
        if (fd < 0) {
            return std::unexpected(ShmError::OpenFailed);
        }

        if (ftruncate(fd, static_cast<off_t>(size)) < 0) {
            close(fd);
            shm_unlink(name);
            return std::unexpected(ShmError::TruncateFailed);
        }

        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            shm_unlink(name);
            return std::unexpected(ShmError::MapFailed);
        }

        std::memset(ptr, 0, size);

        return ShmRegion(ptr, size, fd, true, name);
    }

    [[nodiscard]] static std::expected<ShmRegion, ShmError> open(const char* name, size_t size) {
        const int fd = ::shm_open(name, O_RDWR, 0);
        if (fd < 0) {
            return std::unexpected(ShmError::NotFound);
        }

        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            return std::unexpected(ShmError::MapFailed);
        }

        return ShmRegion(ptr, size, fd, false, name);
    }

    template <typename T>
    [[nodiscard]] T* as() noexcept {
        return static_cast<T*>(data_);
    }

    template <typename T>
    [[nodiscard]] const T* as() const noexcept {
        return static_cast<const T*>(data_);
    }

private:
    ShmRegion(void* data, size_t size, int fd, bool owner, const char* name) noexcept
        : data_(data)
        , size_(size)
        , fd_(fd)
        , owner_(owner)
        , path_(name) {}

    void cleanup() noexcept {
        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, size_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
        if (owner_ && !path_.empty()) {
            (void)::shm_unlink(path_.c_str());
        }
    }

    void* data_{nullptr};
    size_t size_{0};
    int fd_{-1};
    bool owner_{false};
    std::string path_;
};

// ============ Writer ============

class TalosDataWriter {
public:
    ~TalosDataWriter() noexcept { finalize_write_if_needed(); }

    TalosDataWriter(TalosDataWriter&& other) noexcept
        : region_(std::move(other.region_))
        , buffer_(&region_.as<TalosShmTriple>()->buffer)
        , write_in_progress_(std::exchange(other.write_in_progress_, false))
        , write_slot_(std::exchange(other.write_slot_, 0)) {}

    TalosDataWriter& operator=(TalosDataWriter&& other) noexcept {
        if (this != &other) {
            finalize_write_if_needed();
            region_ = std::move(other.region_);
            buffer_ = TripleBufferOps<TalosTripleBuffer, chrial::TalosData>(
                &region_.as<TalosShmTriple>()->buffer);
            write_in_progress_ = std::exchange(other.write_in_progress_, false);
            write_slot_        = std::exchange(other.write_slot_, 0);
        }
        return *this;
    }

    TalosDataWriter(const TalosDataWriter&)            = delete;
    TalosDataWriter& operator=(const TalosDataWriter&) = delete;

    [[nodiscard]] static std::expected<TalosDataWriter, ShmError> create() {
        auto region = ShmRegion::create(TALOS_SHM_NAME, TALOS_SHM_SIZE);
        if (!region) {
            return std::unexpected(region.error());
        }

        auto* shm           = region->as<TalosShmTriple>();
        shm->header.magic   = TALOS_SHM_MAGIC;
        shm->header.version = TALOS_SHM_VERSION;

        return TalosDataWriter(std::move(*region));
    }

    /**
     * @brief 写入并发布
     */
    void write(const chrial::TalosData& data) noexcept {
        begin_write_if_needed();
        auto* shm                      = region_.as<TalosShmTriple>();
        shm->buffer.slots[write_slot_] = data;
        finalize_write_if_needed();
        buffer_.publish();
    }

private:
    void begin_write_if_needed() noexcept {
        if (write_in_progress_) {
            return;
        }

        auto* shm   = region_.as<TalosShmTriple>();
        write_slot_ = shm->buffer.write_idx;
        shm->buffer.slot_seq[write_slot_].fetch_add(1, std::memory_order_seq_cst); // odd
        write_in_progress_ = true;
    }

    void finalize_write_if_needed() noexcept {
        if (!write_in_progress_) {
            return;
        }

        auto* shm = region_.as<TalosShmTriple>();
        shm->buffer.slot_seq[write_slot_].fetch_add(1, std::memory_order_seq_cst); // even
        write_in_progress_ = false;
    }

    explicit TalosDataWriter(ShmRegion&& region) noexcept
        : region_(std::move(region))
        , buffer_(&region_.as<TalosShmTriple>()->buffer) {}

    ShmRegion region_;
    TripleBufferOps<TalosTripleBuffer, chrial::TalosData> buffer_;
    bool write_in_progress_{false};
    uint8_t write_slot_{0};
};

// ============ Reader ============

class TalosDataReader {
public:
    ~TalosDataReader() = default;

    TalosDataReader(TalosDataReader&&)                 = default;
    TalosDataReader& operator=(TalosDataReader&&)      = default;
    TalosDataReader(const TalosDataReader&)            = delete;
    TalosDataReader& operator=(const TalosDataReader&) = delete;

    [[nodiscard]] static std::expected<TalosDataReader, ShmError> open() {
        auto region = ShmRegion::open(TALOS_SHM_NAME, TALOS_SHM_SIZE);
        if (!region) {
            return std::unexpected(region.error());
        }

        auto* shm = region->as<TalosShmTriple>();
        if (shm->header.magic != TALOS_SHM_MAGIC) {
            return std::unexpected(ShmError::InvalidMagic);
        }
        if (shm->header.version != TALOS_SHM_VERSION) {
            return std::unexpected(ShmError::VersionMismatch);
        }

        return TalosDataReader(std::move(*region));
    }

    /**
     * @brief 尝试读取新数据
     */
    [[nodiscard]] std::optional<chrial::TalosData> read_new() noexcept {
        auto result = buffer_.borrow();
        if (!result) {
            return std::nullopt;
        }

        const auto* shm   = region_.as<TalosShmTriple>();
        const auto* slots = &shm->buffer.slots[0];
        const auto diff   = *result - slots;
        if (diff < 0 || diff >= 3) {
            return std::nullopt;
        }

        return copy_consistent_slot(static_cast<uint8_t>(diff));
    }

    /**
     * @brief 读取最新数据（即使已读）
     */
    [[nodiscard]] chrial::TalosData read_latest() const noexcept {
        const auto* shm = region_.as<TalosShmTriple>();
        return copy_consistent_slot(shm->buffer.read_idx);
    }

private:
    [[nodiscard]] chrial::TalosData copy_consistent_slot(uint8_t slot_idx) const noexcept {
        const auto* shm = region_.as<TalosShmTriple>();
        chrial::TalosData snapshot{};
        while (true) {
            const uint64_t begin = shm->buffer.slot_seq[slot_idx].load(std::memory_order_acquire);
            if (begin & 1ULL) {
                continue;
            }

            snapshot = shm->buffer.slots[slot_idx];

            std::atomic_thread_fence(std::memory_order_acquire);
            const uint64_t end = shm->buffer.slot_seq[slot_idx].load(std::memory_order_acquire);
            if (begin == end && !(end & 1ULL)) {
                return snapshot;
            }
        }
    }

    explicit TalosDataReader(ShmRegion&& region) noexcept
        : region_(std::move(region))
        , buffer_(&region_.as<TalosShmTriple>()->buffer) {}

    ShmRegion region_;
    TripleBufferOps<TalosTripleBuffer, chrial::TalosData> buffer_;
};

} // namespace talos::chiral::ipc
