#pragma once

// CHRIAL: bridge between talos and ROS2

#include <array>
#include <cstdint>

namespace talos::chrial {

// 编译期 Tag, 用于标注类型, 0运行时开销
template <typename T>
concept tag = requires { sizeof(T) == 0; };

using Radian       = double;
using RadianPerSec = double;
using Meter        = double;

struct odom {};
struct gimbal_yaw {};
struct gimbal_pitch {};
struct camera {};
struct muzzle {};
struct untyped {};

// 编译期 Tag 结束

template <typename T>
struct timestamped {
    uint64_t timestamp_ns_system_clock;
    T data;
};

// 防止 ABI 踩坑

template <tag From, tag To = untyped>
struct Vector3d {
    double x, y, z;
};

template <tag From, tag To = untyped>
struct Quateriond {
    double x, y, z, w;
};

template <tag From, tag To = untyped>
struct Transform {
    Vector3d<From, To> translation;
    Quateriond<From, To> rotation;
};

enum TargetStateKind : uint8_t { Robot = 0, Outpost = 1 };

enum class TrackerStatus : uint8_t {
    Idle      = 0,
    Detecting = 1,
    Tracking  = 2,
    TempLost  = 3,
};

enum class ArmorColor : uint8_t {
    Blue    = 0,
    Red     = 1,
    Neutral = 2,
    Purple  = 3,
};

enum class ArmorName : uint8_t {
    Sentry = 0,
    One,
    Two,
    Three,
    Four,
    Five,
    Outpost,
    Base,
    BaseLarge,
    Invalid
};

struct OutpostState {
    Vector3d<gimbal_yaw> position;
    Vector3d<gimbal_yaw> velocity;
    Radian yaw;
    RadianPerSec v_yaw;
    // odom z0, z1, z2
    std::array<double, 3> z{0, 0, 0};
};

struct RobotState {
    Vector3d<gimbal_yaw> position;
    Vector3d<gimbal_yaw> velocity;
    Radian yaw;
    RadianPerSec v_yaw;
    Meter radius0; // Armor 0,2 radius
    Meter radius1; // Armor 1,3 radius
    Meter z1;      // z0 + h (armor 1,3 height)
    uint32_t armor_num;
};

struct TargetState {
    TrackerStatus status;
    ArmorColor color;
    ArmorName name;
    RobotState robot;
    OutpostState outpost;
};

struct TalosData {
    TargetStateKind state_kind;
    TargetState state;

    Transform<odom, gimbal_yaw> gimbal_link;
    Transform<gimbal_yaw, muzzle> muzzle_link;
    Transform<gimbal_yaw, camera> camera_link;
};

// ============ Incoming data (External → Talos) ============

struct IncomingData {
    uint64_t timestamp_ns = 0;
};

} // namespace talos::chrial
