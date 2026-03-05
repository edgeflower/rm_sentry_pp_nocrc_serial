#pragma once
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_sentry_pp {

#pragma pack(push, 1)

static constexpr uint8_t ID_IMU = 0x10;
static constexpr uint8_t ID_ROBOT_CMD = 0x11;

struct HeaderFrame {
    static constexpr uint8_t SoF() { return 0x5A; }
    static constexpr uint8_t EoF() { return 0xA5; }

    uint8_t sof; // 0x5A
    uint8_t data_len; // sizeof(data)
    uint8_t id; // 0x10 / 0x11
};

struct ReceiveImuData {
    HeaderFrame frame_header; // id=0x10
    uint32_t time_stamp;

    struct {
        uint8_t self_color; // 0=红色，1=蓝色
        float yaw; // rad
        float pitch; // rad
        float roll; // rad

        float yaw_vel; // rad/s
        float pitch_vel; // rad/s
        float roll_vel; // rad/s
    } data;

    uint8_t eof; // 0xA5
};

struct SendRobotCmdData {
    HeaderFrame frame_header; // id=0x11
    uint32_t time_stamp;

    struct {
        struct {
            float vx;
            float vy;
            float wz;
        } speed_vector;
        struct {
            float yaw_vel;
        } gimbal_big;
    } data;

    uint8_t eof; // 0xA5
};

#pragma pack(pop)

static_assert(sizeof(HeaderFrame) == 3);
static_assert(sizeof(ReceiveImuData) == 33); // 3 + 4 + 24 + 1
static_assert(sizeof(SendRobotCmdData) == 24); // 3 + 4 + 16 + 1

template <typename T>
inline std::vector<uint8_t> toVector(const T& obj)
{
    std::vector<uint8_t> v(sizeof(T));
    std::memcpy(v.data(), &obj, sizeof(T));
    return v;
}

template <typename T>
inline T fromBytes(const uint8_t* p)
{
    T obj {};
    std::memcpy(&obj, p, sizeof(T));
    return obj;
}

template <typename T>
inline void fillHeader(T& pkt, uint8_t id)
{
    pkt.frame_header.sof = HeaderFrame::SoF();
    pkt.frame_header.id = id;
    pkt.frame_header.data_len = static_cast<uint8_t>(sizeof(pkt.data));
    pkt.eof = HeaderFrame::EoF();
}

} // namespace rm_sentry_pp
