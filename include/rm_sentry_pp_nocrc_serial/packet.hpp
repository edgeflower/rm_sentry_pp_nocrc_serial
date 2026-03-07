#pragma once
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_sentry_pp {

#pragma pack(push, 1)

static constexpr uint8_t ID_ROBOT_INFO = 0x06;
static constexpr uint8_t ID_GAME_STATUS = 0x07;
static constexpr uint8_t ID_ALL_ROBOT_HP = 0x08;
static constexpr uint8_t ID_ROBOT_LOCATION = 0x09;
static constexpr uint8_t ID_IMU = 0x10;
static constexpr uint8_t ID_ROBOT_CMD = 0x11;
static constexpr uint8_t ID_ROBOT_POSTURE = 0x12;

struct HeaderFrame {
    static constexpr uint8_t SoF() { return 0x5A; }
    static constexpr uint8_t EoF() { return 0xA5; }

    uint8_t sof; // 0x5A
    uint8_t data_len; // sizeof(data)
    uint8_t id; // 0x10 / 0x11
};



// 机器人信息数据包  0x0201 
struct ReceiveRobotInfoData
{
    HeaderFrame frame_header;  // id = 0x06

    uint32_t time_stamp;


    /// @brief 机器人裁判系统信息  bytes
    struct
    {
        uint8_t id;     // 机器人id
        uint8_t color;  // 0-red 1-blue 2-unknown
        bool attacked;  // 是否被击打
        uint16_t hp;    // 机器人剩余血量
        uint16_t heat;  // 机器人枪管热量
        uint16_t projectile_allowance_17mm; // 17mm弹丸剩余量
        uint16_t posture; // 机器人姿态   1 进攻 、2 防御 、 3 移动
    } data;  // 裁判系统信息

    uint8_t eof; // 0xA5

};


// 比赛信息数据包   0x0001
struct ReceiveGameStatusData
{
    HeaderFrame frame_header;  // id = 0x07

    uint32_t time_stamp;

    struct
    {
        uint8_t game_progress;        // 当前比赛阶段
        uint16_t stage_remain_time;   // 当前阶段剩余时间
    } data;

    uint8_t eof; // 0xA5
};





// 全场机器人hp信息数据包  0x0003
struct ReceiveAllRobotHpData
{
    HeaderFrame frame_header;  // id = 0x08

    uint32_t time_stamp;

    struct
    {
        uint16_t red_1_robot_hp;  // 红方英雄机器人血量
        uint16_t red_2_robot_hp;  // 红方工程机器人血量
        uint16_t red_3_robot_hp;  // 红方步兵机器人血量
        uint16_t red_4_robot_hp;  // 红方步兵机器人血量
        uint16_t red_5_robot_hp;  // 保留位
        uint16_t red_7_robot_hp;  // 红方哨兵机器人血量
        uint16_t red_outpost_hp;  // 红方前哨站血量
        uint16_t red_base_hp;     // 红方基地血量
        uint16_t blue_1_robot_hp; // 蓝方英雄机器人血量
        uint16_t blue_2_robot_hp; // 蓝方工程机器人血量
        uint16_t blue_3_robot_hp; // 蓝方步兵机器人血量
        uint16_t blue_4_robot_hp; // 蓝方步兵机器人血量
        uint16_t blue_5_robot_hp; // 保留位
        uint16_t blue_7_robot_hp; // 蓝方哨兵机器人血量
        uint16_t blue_outpost_hp; // 蓝方前哨站血量
        uint16_t blue_base_hp;    // 蓝方基地血量
    } data;

    uint8_t eof; // 0xA5

};






struct ReceiveRobotLocation    // 机器人位置 0x020B
{
    HeaderFrame frame_header;  // id = 0x09

    uint32_t time_stamp;

    struct
    {
          float hero_x;         
          float hero_y;  
          float engineer_x;  
          float engineer_y;  
          float standard_3_x;  // 3号步兵位置
          float standard_3_y;  
          float standard_4_x;  // 4号步兵位置
          float standard_4_y;  
          float sentry_x;  // 哨兵位置
          float sentry_y;  // 哨兵位置

    } data;

    uint8_t eof; // 0xA5

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

struct SendRobotPostureData   // 机器人姿态 0x0120
{
    HeaderFrame frame_header;  // id = 0x12

    uint32_t time_stamp;

    struct
    {
        uint16_t posture; // 机器人姿态    1 进攻 、2 防御 、 3 移动
    } data;

    uint8_t eof; // 0xA5

};

#pragma pack(pop)

static_assert(sizeof(HeaderFrame) == 3);
static_assert(sizeof(ReceiveRobotInfoData) == 19);    // 3 + 4 + 12 + 1
static_assert(sizeof(ReceiveGameStatusData) == 11);   // 3 + 4 + 3 + 1
static_assert(sizeof(ReceiveAllRobotHpData) == 40);   // 3 + 4 + 33 + 1
static_assert(sizeof(ReceiveRobotLocation) == 48);    // 3 + 4 + 40 + 1
static_assert(sizeof(ReceiveImuData) == 33);          // 3 + 4 + 24 + 1
static_assert(sizeof(SendRobotCmdData) == 24);        // 3 + 4 + 16 + 1
static_assert(sizeof(SendRobotPostureData) == 10);    // 3 + 4 + 2 + 1

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
