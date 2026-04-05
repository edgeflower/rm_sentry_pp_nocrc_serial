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
static constexpr uint8_t ID_RFID = 0x13;

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
        uint16_t heat_limit; // 机器人热量限制
        uint16_t shot_allowance; // 17mm弹丸剩余量
        uint16_t posture; // 机器人实际姿态 1=进攻, 2=防御, 3=移动 (下位机控制)
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

struct ReceiveRFID
{   
    HeaderFrame frame_header;  // id = 0x13

    uint32_t time_stamp;
   
    struct{
        // ---- 基础点 ----
        bool base_self;           // bit0 己方基地增益点
        bool highland_self;       // bit1 己方中央高地增益点
        bool highland_enemy;      // bit2 对方中央高地增益点
        bool slope_self;          // bit3 己方梯形高地增益点
        bool slope_enemy;         // bit4 对方梯形高地增益点

        // ---- 飞坡 ----
        bool fly_self_front;      // bit5 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
        bool fly_self_back;       // bit6 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
        bool fly_enemy_front;     // bit7 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
        bool fly_enemy_back;      // bit8 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）

        // ---- 中央高地地形跨越 ----
        bool center_low_self;     // bit9  己方地形跨越增益点（中央高地下方）
        bool center_high_self;    // bit10 己方地形跨越增益点（中央高地上方）
        bool center_low_enemy;    // bit11 对方地形跨越增益点（中央高地下方）
        bool center_high_enemy;   // bit12 对方地形跨越增益点（中央高地上方）

        // ---- 公路 ----
        bool road_low_self;       // bit13 己方地形跨越增益点（公路下方）
        bool road_high_self;      // bit14 己方地形跨越增益点（公路上方）
        bool road_low_enemy;      // bit15 对方地形跨越增益点（公路下方）
        bool road_high_enemy;     // bit16 对方地形跨越增益点（公路上方）

        // ---- 战略点 ----
        bool fortress_self;       // bit17 己方堡垒增益点
        bool outpost_self;        // bit18 己方前哨站增益点
        bool resource_isolated;   // bit19 己方与资源区不重叠的补给区/RMUL 补给区
        bool resource_overlap;    // bit20 己方与资源区重叠的补给区
        bool supply_self;         // bit21 己方装配增益点
        bool supply_enemy;        // bit22 对方装配增益点
        bool center_bonus;        // bit23 中心增益点（仅 RMUL 适用）

        // ---- 敌方点 ----
        bool fortress_enemy;      // bit24 对方堡垒增益点
        bool outpost_enemy;       // bit25 对方前哨站增益点

        // ---- 隧道（己方）----
        bool tunnel_self_1;       // bit26 己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
        bool tunnel_self_2;       // bit27 己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
        bool tunnel_self_3;       // bit28 己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
        bool tunnel_self_4;       // bit29 己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
        bool tunnel_self_5;       // bit30 己方地形跨越增益点（隧道）（靠近己方梯形高地较中间）
        bool tunnel_self_6;       // bit31 己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）

        // ---- 隧道（敌方）----
        bool tunnel_enemy_1;      // bit32 对方地形跨越增益点（隧道）（靠近对方公路一侧下方）
        bool tunnel_enemy_2;      // bit33 对方地形跨越增益点（隧道）（靠近对方公路一侧中间）
        bool tunnel_enemy_3;      // bit34 对方地形跨越增益点（隧道）（靠近对方公路一侧上方）
        bool tunnel_enemy_4;      // bit35 对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
        bool tunnel_enemy_5;      // bit36 对方地形跨越增益点（隧道）（靠近对方梯形高地较中间）
        bool tunnel_enemy_6;      // bit37 对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）

    }data;

    uint8_t eof; // 0xA5
};

struct ReceiveRfid {
    HeaderFrame frame_header; // id=0x13
    uint32_t time_stamp;

    struct {
        uint32_t rfid_status;     // bit0 ~ bit31
        uint8_t  rfid_status_2;   // bit32 ~ bit39（扩展位）
    } data;

    uint8_t eof; // 0xA5
};

struct ReceiveImuData {
    HeaderFrame frame_header; // id=0x10
    uint32_t time_stamp;

    struct {
        uint8_t self_color; // 0=红色，1=蓝色
        float gimbal_yaw; // 小云台 gimbal_yaw 的机械角 与上电时偏移角度（正中心） 单位 ° ，而不是 gimbal_big 的机械角，gimbal_big 只有 imu 的角度
        float chassis_yaw; // 底盘位姿 下位机计算出来的 范围 -π ~ π // 这玩意 用不上，先不用
        float yaw; // rad  这是 gimbal_big 的 imu 角度 ，而 gimbal_yaw的imu 角度 在 视觉给的 imu 信息里
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
            float yaw_vel;  // 大云台转动速度，rad/s
            float yaw_angle; // 大云台转动角度，rad
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
        uint16_t follow_gimbal_big; // 是否跟随大云台 0 不跟随 1 跟随  //底盘跟随
        //uint16_t track_status;      // 是否启动履带 0 不启动 1 启动
    } data;

    uint8_t eof; // 0xA5

};

#pragma pack(pop)

static_assert(sizeof(HeaderFrame) == 3);
static_assert(sizeof(ReceiveRobotInfoData) == 21);    // 3 + 4 + 14 + 1
static_assert(sizeof(ReceiveGameStatusData) == 11);   // 3 + 4 + 3 + 1
static_assert(sizeof(ReceiveAllRobotHpData) == 40);   // 3 + 4 + 33 + 1
static_assert(sizeof(ReceiveRobotLocation) == 48);    // 3 + 4 + 40 + 1
static_assert(sizeof(ReceiveImuData) == 41);          // 3 + 4 + 28 + 1
static_assert(sizeof(SendRobotCmdData) == 28);        // 3 + 4 + 16 + 1
static_assert(sizeof(SendRobotPostureData) == 12);    // 3 + 4 + 6 + 1
static_assert(sizeof(ReceiveRfid) == 13 );            // 3 + 4 + 38 + 1
static_assert(sizeof(ReceiveRFID) == 46 );            // 3 + 4 + 5 + 1

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
