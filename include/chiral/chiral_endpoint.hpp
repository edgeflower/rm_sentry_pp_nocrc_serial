#pragma once

#include "shm_layout.hpp"
#include "shm_triple_buffer.hpp"
#include <atomic>
#include <cstring>
#include <expected>
#include <fcntl.h>
#include <memory>
#include <optional>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
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

// ============ 常量 ============

inline constexpr uint32_t TALOS_SHM_MAGIC   = 0x544C4454; // "TLDT"
inline constexpr uint32_t TALOS_SHM_VERSION = 2;          // v2: triple buffer

// ============ SHM 名称策略 ============

template <typename T>
struct ShmName;

// ============ SHM 布局 ============

struct alignas(64) ShmHeader {
    uint32_t magic;
    uint32_t version;
    uint8_t _pad[56];
};
static_assert(sizeof(ShmHeader) == 64);

template <typename T>
struct ShmLayout {
    ShmHeader header;
    TripleBufferLayout<T> buffer;
};

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
        // Crash recovery: try to open existing first
        int fd = ::shm_open(name, O_RDWR, 0);
        if (fd >= 0) {
            struct stat st{};
            if (fstat(fd, &st) == 0 && static_cast<size_t>(st.st_size) >= size) {
                void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
                if (ptr != MAP_FAILED) {
                    return ShmRegion(ptr, size, fd, false, name);
                }
            }
            close(fd);
            shm_unlink(name);
        }

        fd = ::shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0644);
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

    [[nodiscard]] bool is_current_mapping(const char* name) const noexcept {
        if (fd_ < 0) {
            return false;
        }

        struct stat current{};
        if (fstat(fd_, &current) < 0) {
            return false;
        }

        const int named_fd = ::shm_open(name, O_RDWR, 0);
        if (named_fd < 0) {
            return false;
        }

        struct stat named{};
        const bool ok = fstat(named_fd, &named) == 0 && current.st_dev == named.st_dev
                     && current.st_ino == named.st_ino;
        close(named_fd);
        return ok;
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

// ============ 内部: 单向通道 (Writer) ============

template <typename T>
class ChannelWriter {
public:
    ~ChannelWriter() noexcept { finalize_write_if_needed(); }

    ChannelWriter(ChannelWriter&& other) noexcept
        : region_(std::move(other.region_))
        , buffer_(&region_.template as<ShmLayout<T>>()->buffer)
        , write_in_progress_(std::exchange(other.write_in_progress_, false))
        , write_slot_(std::exchange(other.write_slot_, 0)) {}

    ChannelWriter& operator=(ChannelWriter&& other) noexcept {
        if (this != &other) {
            finalize_write_if_needed();
            region_ = std::move(other.region_);
            buffer_ = TripleBufferOps<TripleBufferLayout<T>, T>(
                &region_.template as<ShmLayout<T>>()->buffer);
            write_in_progress_ = std::exchange(other.write_in_progress_, false);
            write_slot_        = std::exchange(other.write_slot_, 0);
        }
        return *this;
    }

    ChannelWriter(const ChannelWriter&)            = delete;
    ChannelWriter& operator=(const ChannelWriter&) = delete;

    [[nodiscard]] static std::expected<ChannelWriter, ShmError> create() {
        constexpr size_t shm_size = sizeof(ShmLayout<T>);
        auto region               = ShmRegion::create(ShmName<T>::value, shm_size);
        if (!region) {
            return std::unexpected(region.error());
        }

        auto* shm           = region->template as<ShmLayout<T>>();
        shm->header.magic   = TALOS_SHM_MAGIC;
        shm->header.version = TALOS_SHM_VERSION;

        return ChannelWriter(std::move(*region));
    }

    void write(const T& data) noexcept {
        begin_write_if_needed();
        auto* shm                      = region_.template as<ShmLayout<T>>();
        shm->buffer.slots[write_slot_] = data;
        finalize_write_if_needed();
        buffer_.publish();
    }

private:
    void begin_write_if_needed() noexcept {
        if (write_in_progress_) {
            return;
        }
        auto* shm   = region_.template as<ShmLayout<T>>();
        write_slot_ = shm->buffer.write_idx;
        shm->buffer.slot_seq[write_slot_].fetch_add(1, std::memory_order_seq_cst);
        write_in_progress_ = true;
    }

    void finalize_write_if_needed() noexcept {
        if (!write_in_progress_) {
            return;
        }
        auto* shm = region_.template as<ShmLayout<T>>();
        shm->buffer.slot_seq[write_slot_].fetch_add(1, std::memory_order_seq_cst);
        write_in_progress_ = false;
    }

    explicit ChannelWriter(ShmRegion&& region) noexcept
        : region_(std::move(region))
        , buffer_(&region_.template as<ShmLayout<T>>()->buffer) {}

    ShmRegion region_;
    TripleBufferOps<TripleBufferLayout<T>, T> buffer_;
    bool write_in_progress_{false};
    uint8_t write_slot_{0};
};

// ============ 内部: 单向通道 (Reader) ============

template <typename T>
class ChannelReader {
public:
    ~ChannelReader() = default;

    ChannelReader(ChannelReader&&)                 = default;
    ChannelReader& operator=(ChannelReader&&)      = default;
    ChannelReader(const ChannelReader&)            = delete;
    ChannelReader& operator=(const ChannelReader&) = delete;

    [[nodiscard]] static std::expected<ChannelReader, ShmError> open() {
        constexpr size_t shm_size = sizeof(ShmLayout<T>);
        auto region               = ShmRegion::open(ShmName<T>::value, shm_size);
        if (!region) {
            return std::unexpected(region.error());
        }

        auto* shm = region->template as<ShmLayout<T>>();
        if (shm->header.magic != TALOS_SHM_MAGIC) {
            return std::unexpected(ShmError::InvalidMagic);
        }
        if (shm->header.version != TALOS_SHM_VERSION) {
            return std::unexpected(ShmError::VersionMismatch);
        }

        return ChannelReader(std::move(*region));
    }

    [[nodiscard]] std::optional<T> read_new() noexcept {
        auto result = buffer_.borrow();
        if (!result) {
            return std::nullopt;
        }

        auto* shm   = region_.template as<ShmLayout<T>>();
        auto* slots = &shm->buffer.slots[0];
        auto diff   = *result - slots;
        if (diff < 0 || diff >= 3) {
            return std::nullopt;
        }

        return copy_consistent_slot(static_cast<uint8_t>(diff));
    }

    [[nodiscard]] T read_latest() const noexcept {
        auto* shm = region_.template as<ShmLayout<T>>();
        return copy_consistent_slot(shm->buffer.read_idx);
    }

    [[nodiscard]] bool is_current_mapping() const noexcept {
        return region_.is_current_mapping(ShmName<T>::value);
    }

private:
    [[nodiscard]] T copy_consistent_slot(uint8_t slot_idx) const noexcept {
        auto* shm = region_.template as<ShmLayout<T>>();
        T snapshot{};
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

    explicit ChannelReader(ShmRegion&& region) noexcept
        : region_(std::move(region))
        , buffer_(&region_.template as<ShmLayout<T>>()->buffer) {}

    ShmRegion region_;
    TripleBufferOps<TripleBufferLayout<T>, T> buffer_;
};

// ============ 双向端点 ============

/**
 * @brief 通用双向 IPC 端点
 *
 * 模板参数决定方向：
 *   - Outgoing: 此端点作为 PRODUCER 写入的类型
 *   - Incoming: 此端点作为 CONSUMER 读取的类型
 *
 * 类型安全保证：
 *   - write() 只接受 Outgoing 类型
 *   - read_new() 只返回 Incoming 类型
 *   - 编译器阻止写错方向
 *
 * 用法：
 *   using TalosSide  = ChiralEndpoint<TalosData, IncomingData>;
 *   using RemoteSide = ChiralEndpoint<IncomingData, TalosData>;
 *
 *   // Talos 端：create outgoing; incoming is opened lazily on read
 *   auto talos = TalosSide::create();
 *
 *   // External 端：create outgoing; incoming is opened lazily on read
 *   auto remote = RemoteSide::create();
 */
template <typename Outgoing, typename Incoming>
class ChiralEndpoint {
public:
    ~ChiralEndpoint() = default;

    ChiralEndpoint(ChiralEndpoint&&)                 = default;
    ChiralEndpoint& operator=(ChiralEndpoint&&)      = default;
    ChiralEndpoint(const ChiralEndpoint&)            = delete;
    ChiralEndpoint& operator=(const ChiralEndpoint&) = delete;

    /**
     * @brief 创建本端写入通道
     *
     * 创建 outgoing SHM (此端为 producer)。incoming SHM 由 read_new()
     * / read_latest() 按需打开，并在对端重启后自动重连。
     */
    [[nodiscard]] static std::expected<std::unique_ptr<ChiralEndpoint>, ShmError>
        create() noexcept {
        auto writer = ChannelWriter<Outgoing>::create();
        if (!writer) {
            return std::unexpected(writer.error());
        }

        return std::make_unique<ChiralEndpoint>(std::move(*writer));
    }

    void write(const Outgoing& data) noexcept { writer_.write(data); }

    [[nodiscard]] std::optional<Incoming> read_new() noexcept {
        auto* reader = lazy_reader();
        if (reader == nullptr) {
            return std::nullopt;
        }

        auto data = reader->read_new();
        if (data || reader->is_current_mapping()) {
            return data;
        }

        reader_.reset();
        reader = lazy_reader();
        if (reader == nullptr) {
            return std::nullopt;
        }
        return reader->read_new();
    }

    [[nodiscard]] std::optional<Incoming> read_latest() const noexcept {
        auto* reader = lazy_reader();
        if (reader == nullptr) {
            return std::nullopt;
        }
        return reader->read_latest();
    }

    explicit ChiralEndpoint(ChannelWriter<Outgoing>&& writer) noexcept
        : writer_(std::move(writer))
        , reader_(std::nullopt) {}

private:
    [[nodiscard]] ChannelReader<Incoming>* lazy_reader() const noexcept {
        if (reader_ && !reader_->is_current_mapping()) {
            reader_.reset();
        }
        if (reader_) {
            return &*reader_;
        }

        auto reader = ChannelReader<Incoming>::open();
        if (!reader) {
            return nullptr;
        }

        reader_.emplace(std::move(*reader));
        return &*reader_;
    }

    ChannelWriter<Outgoing> writer_;
    mutable std::optional<ChannelReader<Incoming>> reader_;
};

} // namespace talos::chiral::ipc
