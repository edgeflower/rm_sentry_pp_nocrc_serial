#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_decision_interfaces/msg/detail/friend_location__struct.hpp>
#include <rm_decision_interfaces/msg/detail/rfid__struct.hpp>
#include <rm_decision_interfaces/msg/detail/rfid_parse__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sys/types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <armor_interfaces/msg/target.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "rm_sentry_pp_nocrc_serial/packet.hpp"
#include "rm_sentry_pp_nocrc_serial/serial_port.hpp"
#include "chiral/talos_triple_buffer_shm.hpp"
#include <rm_decision_interfaces/msg/robot_control.hpp>
#include <rm_decision_interfaces/msg/sentry_posture_cmd.hpp>
#include <rm_decision_interfaces/msg/sentry_posture_status.hpp>
#include <rm_decision_interfaces/srv/set_sentry_posture.hpp>
#include <rm_decision_interfaces/msg/all_robot_hp.hpp>
#include <rm_decision_interfaces/msg/game_status.hpp>
#include <rm_decision_interfaces/msg/friend_location.hpp>
#include <rm_decision_interfaces/msg/robot_location.hpp>
#include <rm_decision_interfaces/msg/robot_status.hpp>
#include<rm_decision_interfaces/msg/rfid.hpp>
#include<rm_decision_interfaces/msg/rfid_parse.hpp>

#include "chiral/talos_triple_buffer_shm.hpp"

#include <tf2_ros/transform_broadcaster.h>
using namespace std::chrono_literals;

namespace rm_sentry_pp_nocrc_serial {

class Node : public rclcpp::Node {
public:
    explicit Node(const rclcpp::NodeOptions& options)
        : rclcpp::Node("rm_sentry_pp_nocrc_serial", options)
    {
        port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
        baud_ = declare_parameter<int>("baud", 115200);
        imu_frame_ = declare_parameter<std::string>("imu_frame", "gimbal_big");
        cmd_vel_chassis_topic_ = declare_parameter<std::string>("cmd_vel_chassis_topic", "cmd_vel_chassis");
        robot_control_topic_ = declare_parameter<std::string>("robot_control_topic", "robot_control");
        set_posture_service_name_ = declare_parameter<std::string>("set_posture_service_name", "set_sentry_posture");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "imu");
        send_period_ms_ = declare_parameter<int>("send_period_ms", 5);
        enable_dtr_rts_ = declare_parameter<bool>("enable_dtr_rts", true);
        imu_parent_frame_ = declare_parameter<std::string>("imu_parent_frame", "chassis");

        // Target lost prediction parameters
        confidence_decay_lambda_ = declare_parameter<double>("confidence_decay_lambda", 0.5);
        min_confidence_threshold_ = declare_parameter<double>("min_confidence_threshold", 0.3);

        RCLCPP_INFO(get_logger(), "Target lost prediction: lambda=%.2f, min_threshold=%.2f",
                    confidence_decay_lambda_, min_confidence_threshold_);

        // Gimbal angle timeout for tracking
        gimbal_angle_timeout_ms_ = declare_parameter<int>("gimbal_angle_timeout_ms", 300);
        gimbal_follow_path_topic_ = declare_parameter<std::string>("gimbal_follow_path_topic", "plan");
        gimbal_follow_lookahead_ = declare_parameter<double>("gimbal_follow_lookahead", 1.5);

        RCLCPP_INFO(get_logger(), "Gimbal angle follow timeout: %d ms", gimbal_angle_timeout_ms_);

        // 初始化 TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

        // Chiral 目标跟踪数据发布者
        target_tracking_pub_ = create_publisher<armor_interfaces::msg::Target>("target_tracking", 10);
        gimbal_yaw_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("expected_gimbal_yaw", 10);

        cmd_vel_chassis_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_chassis_topic_, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { onCmd(*msg); });

        robot_control_sub_ = create_subscription<rm_decision_interfaces::msg::RobotControl>(
            robot_control_topic_, 10,
            [this](const rm_decision_interfaces::msg::RobotControl::SharedPtr msg) { onRobotControl(*msg); });

        // 全局路径订阅：gimbal_big 跟随路径方向
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            gimbal_follow_path_topic_, 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) { onPath(*msg); });

        // 创建服务服务器替代话题订阅
        set_posture_service_ = create_service<rm_decision_interfaces::srv::SetSentryPosture>(
            set_posture_service_name_,
            [this](const std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Request> request,
                   std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Response> response) {
                handleSetSentryPosture(request, response);
            });
        RCLCPP_INFO(get_logger(), "Service server created: %s", set_posture_service_name_.c_str());

        gimbal_path_timeout_ms_ = declare_parameter<int>("gimbal_path_timeout_ms", 1000);

        node_start_ = this->now();
        last_imu_calibration_time_ = node_start_;
        last_known_target_.last_update_time = node_start_;
        last_path_time_ = node_start_;

        // 初始化 Chiral 读取器
        auto chiral_reader = talos::chiral::ipc::TalosDataReader::open();
        if (chiral_reader) {
            chiral_reader_ = std::make_unique<talos::chiral::ipc::TalosDataReader>(std::move(*chiral_reader));
            RCLCPP_INFO(get_logger(), "Chiral reader initialized successfully");
        } else {
            RCLCPP_WARN(get_logger(), "Failed to initialize chiral reader: %d", static_cast<int>(chiral_reader.error()));
        }

        // Gimbal 路径跟随高频重采样 timer
        gimbal_path_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),  // 50Hz
            [this]() { updateGimbalFromCachedPath(); });

        protect_thread_ = std::thread([this]() { protectLoop(); });
        rx_thread_ = std::thread([this]() { rxLoop(); });
        tx_thread_ = std::thread([this]() { txLoop(); });
        chiral_thread_ = std::thread([this]() { chiralLoop(); });

        RCLCPP_INFO(get_logger(), "rm_sentry_pp_nocrc_serial started.");
    }

    ~Node() override
    {
        exit_.store(true, std::memory_order_relaxed);

        if (tx_thread_.joinable())
            tx_thread_.join();
        if (rx_thread_.joinable())
            rx_thread_.join();
        if (protect_thread_.joinable())
            protect_thread_.join();
        if (chiral_thread_.joinable())
            chiral_thread_.join();

        std::lock_guard<std::mutex> lk(port_mtx_);
        sp_.close();
    }

private:
    uint32_t nowMs() const
    {
        auto dt = (this->now() - node_start_).nanoseconds();
        return static_cast<uint32_t>(dt / 1000000ULL);
    }

    // 回调函数修改
    void onCmd(const geometry_msgs::msg::Twist& msg)
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        current_cmd_state_.data.speed_vector.vx = msg.linear.x;
        current_cmd_state_.data.speed_vector.vy = msg.linear.y;
        //current_cmd_state_.data.speed_vector.wz = 0.0;
        current_cmd_state_.data.speed_vector.wz = target_spin_vel_; // 使用来自 RobotControl 消息的旋转速度，而不是 cmd_vel_chassis 的角速度

        tx_pending_ = true;
        /*
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Twist in: vx=%.3f vy=%.3f wz=%.3f | stored: vx=%d vy=%d wz=%d",
            msg.linear.x, msg.linear.y, msg.angular.z,
            (int)current_cmd_state_.data.speed_vector.vx,
            (int)current_cmd_state_.data.speed_vector.vy,
            (int)current_cmd_state_.data.speed_vector.wz);
            */
    }

    void onRobotControl(const rm_decision_interfaces::msg::RobotControl& msg)
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        current_cmd_state_.data.gimbal_big.yaw_vel = msg.gimbal_big_yaw_vel;
        target_spin_vel_ = msg.chassis_spin_vel;
        follow_gimbal_big_ = msg.follow_gimbal_big;
        tx_pending_ = true;
    }

    void onPath(const nav_msgs::msg::Path& msg)
    {
        if (msg.poses.empty()) return;
        std::lock_guard<std::mutex> lk(path_mtx_);
        cached_path_ = msg;
        last_path_time_ = this->now();
    }

    void updateGimbalFromCachedPath()
    {
        // Path 超时检查
        if ((this->now() - last_path_time_).seconds() * 1000.0 > gimbal_path_timeout_ms_) return;

        nav_msgs::msg::Path local_path;
        {
            std::lock_guard<std::mutex> lk(path_mtx_);
            if (cached_path_.poses.empty()) return;
            local_path = cached_path_;
        }

        // 查找 gimbal_yaw_fake 在 map 下的位姿
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "gimbal_yaw_fake", tf2::TimePointZero);
        } catch (const tf2::TransformException&) {
            return;
        }

        // 找前视点：路径上距机器人 lookahead 距离的点
        double lookahead = gimbal_follow_lookahead_;
        bool found = false;
        double tx, ty;

        for (const auto& pose : local_path.poses) {
            double px = pose.pose.position.x;
            double py = pose.pose.position.y;

            // 变换到 chassis 帧
            double dx = px - tf.transform.translation.x;
            double dy = py - tf.transform.translation.y;
            tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            double cx = dx * std::cos(yaw) + dy * std::sin(yaw);
            double cy = -dx * std::sin(yaw) + dy * std::cos(yaw);
            double dist = std::hypot(cx, cy);

            if (dist >= lookahead) {
                tx = cx;
                ty = cy;
                found = true;
                break;
            }
        }

        if (!found) {
            // 用路径最后一个点
            const auto& last = local_path.poses.back().pose.position;
            double dx = last.x - tf.transform.translation.x;
            double dy = last.y - tf.transform.translation.y;
            tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            tx = dx * std::cos(yaw) + dy * std::sin(yaw);
            ty = -dx * std::sin(yaw) + dy * std::cos(yaw);
        }

        float angle = std::atan2(ty, tx);
        {
            std::lock_guard<std::mutex> lk(tx_mtx_);
            gimbal_big_yaw_angle_ = angle;
        }

        // 发布 debug marker
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "chassis";
        marker.ns = "expected_gimbal_yaw";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        tf2::Quaternion mq;
        mq.setRPY(0, 0, angle);
        marker.pose.orientation = tf2::toMsg(mq);
        marker.scale.x = 0.8;  // 箭头长度
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration(0, 0);
        gimbal_yaw_marker_pub_->publish(marker);
    }

    void handleSetSentryPosture(
        const std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Request> request,
        std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Response> response)
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        current_robot_posture_state_.data.posture = request->posture;
        tx_posture_pending_ = true;

        // TODO: 可以添加更复杂的验证逻辑
        response->accepted = true;
        response->message = "Posture set to " + std::to_string(request->posture);

        RCLCPP_INFO(get_logger(), "SentryPosture service called: posture=%u override=%d",
                    request->posture, request->override_mode);
    }

    // 保护线程：持续监测串口状态，若发现异常则尝试重连

    void protectLoop()
    {
        while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed)) {
            if (!is_port_ok_.load(std::memory_order_relaxed)) {
                std::lock_guard<std::mutex> lk(port_mtx_);
                sp_.close();
                if (sp_.open(port_, baud_)) {
                    if (enable_dtr_rts_)
                        sp_.setDtrRts(true);
                    is_port_ok_.store(true, std::memory_order_relaxed);
                    RCLCPP_INFO(get_logger(), "Opened port: %s @ %d", port_.c_str(), baud_);
                } else {
                    // 不刷屏
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Open failed: %s", port_.c_str());
                }
            }
            std::this_thread::sleep_for(500ms);
        }
    }

    // 接收线程：持续从串口读取数据，解析成帧，并发布 ROS 消息
    void rxLoop()
    {
        std::vector<uint8_t> rxbuf;
        rxbuf.reserve(4096);

        uint8_t tmp[256];

        while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed)) {
            if (!is_port_ok_.load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(50ms);
                continue;
            }

            int n = 0;
            {
                std::lock_guard<std::mutex> lk(port_mtx_);
                n = sp_.readSome(tmp, sizeof(tmp), 20);
            }

            if (n < 0) {
                is_port_ok_.store(false, std::memory_order_relaxed);
                continue;
            }
            if (n == 0)
                continue;

            rxbuf.insert(rxbuf.end(), tmp, tmp + n);
            /*
            if (n > 0) {
                std::stringstream ss;
                for (int i = 0; i < n; ++i)
                    ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)tmp[i] << " ";
                RCLCPP_INFO(get_logger(), "Raw RX: %s", ss.str().c_str());
            }*/
            parseFrames(rxbuf);
        }
    }

    void parseFrames(std::vector<uint8_t>& rxbuf)
    {
        // 无 CRC：必须靠 SoF + data_len + EoF 强同步
        while (true) {
            if (rxbuf.size() < sizeof(rm_sentry_pp::HeaderFrame))
                return;

            // 找 SoF
            size_t sof_pos = 0;
            while (sof_pos < rxbuf.size() && rxbuf[sof_pos] != rm_sentry_pp::HeaderFrame::SoF())
                sof_pos++;
            if (sof_pos > 0) {
                rxbuf.erase(rxbuf.begin(), rxbuf.begin() + sof_pos);
                if (rxbuf.size() < sizeof(rm_sentry_pp::HeaderFrame))
                    return;
            }

            rm_sentry_pp::HeaderFrame hdr {};
            std::memcpy(&hdr, rxbuf.data(), sizeof(hdr));

            // body = time_stamp(4) + payload(data_len) + eof(1)
            const size_t body_len = 4 + static_cast<size_t>(hdr.data_len) + 1;
            const size_t frame_len = sizeof(rm_sentry_pp::HeaderFrame) + body_len;

            // 防御：无 CRC 时必须限制 data_len
            if (hdr.data_len == 0 || hdr.data_len > 64) {
                rxbuf.erase(rxbuf.begin()); // 丢掉这个 SoF，继续找
                continue;
            }

            if (rxbuf.size() < frame_len)
                return;

            // 校验 EoF
            if (rxbuf[frame_len - 1] != rm_sentry_pp::HeaderFrame::EoF()) {
                rxbuf.erase(rxbuf.begin());
                continue;
            }

            // 需要imu数据 与 point_lio的数据通过robot_localization融合 发布平滑高频的里程计数据，供决策系统使用
            // 分发（你当前 pocket 只有 IMU 真正从下位机来）
            if (hdr.id == rm_sentry_pp::ID_IMU) {
                // 强一致性：IMU 的 data_len 必须是
                if (hdr.data_len == sizeof(rm_sentry_pp::ReceiveImuData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveImuData)) {
                    auto imu = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveImuData>(rxbuf.data());
                    publishImu(imu);
                }
            }
            
            if(hdr.id == rm_sentry_pp::ID_ROBOT_INFO){
                if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveRobotInfoData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveRobotInfoData)){
                    auto robot_info = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveRobotInfoData>(rxbuf.data());
                    // 发布机器人信息消息
                    publishRobotInfo(robot_info);
                }
            }
            if(hdr.id == rm_sentry_pp::ID_GAME_STATUS){
                if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveGameStatusData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveGameStatusData)){
                    auto game_status = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveGameStatusData>(rxbuf.data());
                    // 发布比赛状态消息
                    publishGameStatus(game_status);
                }
            }
            if(hdr.id == rm_sentry_pp::ID_ALL_ROBOT_HP){
                if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveAllRobotHpData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveAllRobotHpData)){
                    auto all_robot_hp = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveAllRobotHpData>(rxbuf.data());
                    // 发布所有机器人血量消息
                    publishAllRobotHp(all_robot_hp);
                }
            }
            if(hdr.id == rm_sentry_pp::ID_ROBOT_LOCATION){
                if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveRobotLocation::data) && frame_len == sizeof(rm_sentry_pp::ReceiveRobotLocation)){
                    auto robot_location = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveRobotLocation>(rxbuf.data());
                    // 发布机器人位置信息消息
                    publishRobotLocation(robot_location);
                }
            }
            if(hdr.id == rm_sentry_pp::ID_RFID){
                if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveRfid::data) && frame_len == sizeof(rm_sentry_pp::ReceiveRfid)){
                    auto rfid = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveRfid>(rxbuf.data());
                    // 发布 rfid 信息
                    publishRfid(rfid);
                }
            }

            rxbuf.erase(rxbuf.begin(), rxbuf.begin() + frame_len);
        }
    }

    void publishImu(const rm_sentry_pp::ReceiveImuData& imu_data)
    {
        auto now = this->now();

        // --- 1. 发布 Imu 消息 ---
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = now;
        imu.header.frame_id = imu_frame_;
        gimbal_yaw_ = imu_data.data.gimbal_yaw; // 机械角获得

        // IMU 漂移修正：当云台回中且没有跟踪目标时，使用机械角校准 IMU
        {
            std::lock_guard<std::mutex> lk(tx_mtx_);
            bool has_target = last_known_target_.valid && last_known_target_.confidence > min_confidence_threshold_;

            // 校准条件：云台接近回中 + 无跟踪目标 + 距上次校准超过间隔
            if (std::abs(gimbal_yaw_) < IMU_CALIBRATION_THRESHOLD &&
                !has_target &&
                !is_calibrating_imu_ &&
                (now - last_imu_calibration_time_).seconds() > IMU_CALIBRATION_INTERVAL) {

                // 计算 IMU yaw 与机械角的偏差（机械角作为绝对参考）
                double imu_yaw = imu_data.data.yaw;
                imu_yaw_offset_ = imu_yaw - gimbal_yaw_;
                is_calibrating_imu_ = true;
                last_imu_calibration_time_ = now;

                RCLCPP_DEBUG(get_logger(), "IMU calibrated: offset=%.3f rad (gimbal_yaw=%.3f, imu_yaw=%.3f)",
                            imu_yaw_offset_, gimbal_yaw_, imu_yaw);
            }

            // 有跟踪目标时停止校准（避免运动中误校准）
            if (has_target) {
                is_calibrating_imu_ = false;
            }
        }

        // 应用修正后的角度
        double corrected_yaw = imu_data.data.yaw - imu_yaw_offset_;

        tf2::Quaternion q;
        // 假设 imu_data 里的顺序是 Roll, Pitch, Yaw
        q.setRPY(imu_data.data.roll, imu_data.data.pitch, corrected_yaw);
        imu.orientation = tf2::toMsg(q);

        imu.angular_velocity.x = imu_data.data.roll_vel;
        imu.angular_velocity.y = imu_data.data.pitch_vel;
        imu.angular_velocity.z = imu_data.data.yaw_vel;
        // 先不发布 imu
        imu_pub_->publish(imu);

        /* 去除不必要的tf 变换
        // --- 2. 发布 TF 变换 ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = imu_parent_frame_; // gimbal_big
        t.child_frame_id = imu_frame_; //  "gimbal_big_imu"

        // IMU 通常不提供位置信息，设为 0
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        // 直接使用上面生成的四元数
        t.transform.rotation = imu.orientation;

        tf_broadcaster_->sendTransform(t);*/
    }

    void publishRobotInfo(const rm_sentry_pp::ReceiveRobotInfoData& robot_info_data)
    {
        auto now = this->now();

        rm_decision_interfaces::msg::SentryPostureStatus posture_msg;
        posture_msg.reported_posture = robot_info_data.data.posture;         // 下位机实际姿态
        posture_msg.current_posture = robot_info_data.data.posture;          // 同一个值

        // 发布机器人姿态状态
        static auto posture_pub = create_publisher<rm_decision_interfaces::msg::SentryPostureStatus>("sentry_posture_status", 10);
        posture_pub->publish(posture_msg);

        rm_decision_interfaces::msg::RobotStatus robot_status_msg;
        robot_status_msg.robot_id = robot_info_data.data.id;
        robot_status_msg.team_color = robot_info_data.data.color;
        robot_status_msg.is_attacked = robot_info_data.data.attacked;
        robot_status_msg.current_hp = robot_info_data.data.hp; 
        robot_status_msg.shot_allowance = robot_info_data.data.shot_allowance; // 17mm 小弹丸 允许发射量
        
        // 发布机器人状态消息
        static auto robot_status_pub = create_publisher<rm_decision_interfaces::msg::RobotStatus>("robot_status", 10);
        robot_status_pub->publish(robot_status_msg);
    }

    void publishGameStatus(const rm_sentry_pp::ReceiveGameStatusData& game_status_data)
    {
        auto now = this->now();

        rm_decision_interfaces::msg::GameStatus game_status_msg;
        game_status_msg.game_progress = game_status_data.data.game_progress;
        game_status_msg.stage_remain_time = game_status_data.data.stage_remain_time;
        // 发布比赛状态消息
        static auto game_status_pub = create_publisher<rm_decision_interfaces::msg::GameStatus>("game_status", 10);
        game_status_pub->publish(game_status_msg);
    }

    void publishAllRobotHp(const rm_sentry_pp::ReceiveAllRobotHpData& all_robot_hp_data)
    {
        auto now = this->now();

        rm_decision_interfaces::msg::AllRobotHP all_robot_hp_msg;
        all_robot_hp_msg.red_1_robot_hp = all_robot_hp_data.data.red_1_robot_hp;
        all_robot_hp_msg.red_2_robot_hp = all_robot_hp_data.data.red_2_robot_hp;
        all_robot_hp_msg.red_3_robot_hp = all_robot_hp_data.data.red_3_robot_hp;
        all_robot_hp_msg.red_4_robot_hp = all_robot_hp_data.data.red_4_robot_hp;
        all_robot_hp_msg.red_7_robot_hp = all_robot_hp_data.data.red_7_robot_hp;
        all_robot_hp_msg.red_outpost_hp = all_robot_hp_data.data.red_outpost_hp;
        all_robot_hp_msg.red_base_hp = all_robot_hp_data.data.red_base_hp;
        all_robot_hp_msg.blue_1_robot_hp = all_robot_hp_data.data.blue_1_robot_hp;
        all_robot_hp_msg.blue_2_robot_hp = all_robot_hp_data.data.blue_2_robot_hp;
        all_robot_hp_msg.blue_3_robot_hp = all_robot_hp_data.data.blue_3_robot_hp;
        all_robot_hp_msg.blue_4_robot_hp = all_robot_hp_data.data.blue_4_robot_hp;
        all_robot_hp_msg.blue_7_robot_hp = all_robot_hp_data.data.blue_7_robot_hp;
        all_robot_hp_msg.blue_outpost_hp = all_robot_hp_data.data.blue_outpost_hp;
        all_robot_hp_msg.blue_base_hp = all_robot_hp_data.data.blue_base_hp;
        
        
        // 发布所有机器人血量消息
        static auto all_robot_hp_pub = create_publisher<rm_decision_interfaces::msg::AllRobotHP>("all_robot_hp", 10);
        all_robot_hp_pub->publish(all_robot_hp_msg);
    }

    void publishRobotLocation(const rm_sentry_pp::ReceiveRobotLocation& robot_location_data)
    {
        auto now = this->now();

        rm_decision_interfaces::msg::FriendLocation robot_location_msg;
        robot_location_msg.hero_x = robot_location_data.data.hero_x;
        robot_location_msg.hero_y = robot_location_data.data.hero_y;
        robot_location_msg.engineer_x = robot_location_data.data.engineer_x;
        robot_location_msg.engineer_y = robot_location_data.data.engineer_y;
        robot_location_msg.standard_3_x = robot_location_data.data.standard_3_x;
        robot_location_msg.standard_3_y = robot_location_data.data.standard_3_y;
        robot_location_msg.standard_4_x = robot_location_data.data.standard_4_x;
        robot_location_msg.standard_4_y = robot_location_data.data.standard_4_y;
        robot_location_msg.standard_5_x = robot_location_data.data.sentry_x;
        robot_location_msg.standard_5_y = robot_location_data.data.sentry_y;
        
        

        // 发布机器人位置信息消息
        static auto robot_location_pub = create_publisher<rm_decision_interfaces::msg::FriendLocation>("robot_location", 10);
        robot_location_pub->publish(robot_location_msg);
    }

    void publishRfid(const rm_sentry_pp::ReceiveRfid& rfid_msg){
        auto now = this->now();
        /*
        rm_decision_interfaces::msg::RFID rfid_msg;
        rfid_msg.rfid_status = rfid_data.data.rfid_status;
        rfid_msg.rfid_status_2 = rfid_data.data.rfid_status_2;
        */
        #define GET_BIT(x,n) ((x) >> (n))
        rm_decision_interfaces::msg::RFIDParse rfid_parse;
        // ---- 基础点 ----
        rfid_parse.base_self = GET_BIT(rfid_msg.data.rfid_status , 0);
        rfid_parse.highland_self = GET_BIT(rfid_msg.data.rfid_status , 1);
        rfid_parse.highland_enemy  = GET_BIT(rfid_msg.data.rfid_status , 2);
        rfid_parse.slope_self  = GET_BIT(rfid_msg.data.rfid_status , 3);
        rfid_parse.slope_enemy  = GET_BIT(rfid_msg.data.rfid_status , 4);
        // ---- 飞坡 ----
        rfid_parse.fly_self_front  = GET_BIT(rfid_msg.data.rfid_status , 5);
        rfid_parse.fly_self_back  = GET_BIT(rfid_msg.data.rfid_status , 6);
        rfid_parse.fly_enemy_front  = GET_BIT(rfid_msg.data.rfid_status , 7);
        rfid_parse.fly_enemy_back  = GET_BIT(rfid_msg.data.rfid_status , 8);
        // ---- 中央高地地形跨越 ----
        rfid_parse.center_low_self  = GET_BIT(rfid_msg.data.rfid_status , 9);
        rfid_parse.center_high_self  = GET_BIT(rfid_msg.data.rfid_status , 10);
        rfid_parse.center_low_enemy  = GET_BIT(rfid_msg.data.rfid_status , 11);
        rfid_parse.center_high_enemy  = GET_BIT(rfid_msg.data.rfid_status , 12);
        // ---- 公路 ----
        rfid_parse.road_low_self  = GET_BIT(rfid_msg.data.rfid_status , 13);
        rfid_parse.road_high_self  = GET_BIT(rfid_msg.data.rfid_status , 14);
        rfid_parse.road_low_enemy  = GET_BIT(rfid_msg.data.rfid_status , 15);
        rfid_parse.road_high_enemy  = GET_BIT(rfid_msg.data.rfid_status , 16);
        // ---- 战略点 ----
        rfid_parse.fortress_self  = GET_BIT(rfid_msg.data.rfid_status , 17);
        rfid_parse.outpost_self  = GET_BIT(rfid_msg.data.rfid_status , 18);
        rfid_parse.resource_isolated  = GET_BIT(rfid_msg.data.rfid_status , 19);
        rfid_parse.resource_overlap  = GET_BIT(rfid_msg.data.rfid_status , 20);
        rfid_parse.supply_self  = GET_BIT(rfid_msg.data.rfid_status , 21);
        rfid_parse.supply_enemy  = GET_BIT(rfid_msg.data.rfid_status , 22);
        rfid_parse.center_bonus  = GET_BIT(rfid_msg.data.rfid_status , 23);
        // ---- 敌方点 ----
        rfid_parse.fortress_enemy  = GET_BIT(rfid_msg.data.rfid_status , 24);
        rfid_parse.outpost_enemy  = GET_BIT(rfid_msg.data.rfid_status , 25);
        // ---- 隧道（己方）----
        rfid_parse.tunnel_self_1  = GET_BIT(rfid_msg.data.rfid_status , 26);
        rfid_parse.tunnel_self_2  = GET_BIT(rfid_msg.data.rfid_status , 27);
        rfid_parse.tunnel_self_3  = GET_BIT(rfid_msg.data.rfid_status , 28);
        rfid_parse.tunnel_self_4  = GET_BIT(rfid_msg.data.rfid_status , 29);
        rfid_parse.tunnel_self_5  = GET_BIT(rfid_msg.data.rfid_status , 30);
        rfid_parse.tunnel_self_6  = GET_BIT(rfid_msg.data.rfid_status , 31);
        // ---- 隧道（敌方）----
        rfid_parse.tunnel_enemy_1  = GET_BIT(rfid_msg.data.rfid_status_2 ,0 );
        rfid_parse.tunnel_enemy_2  = GET_BIT(rfid_msg.data.rfid_status_2 ,1 );
        rfid_parse.tunnel_enemy_3  = GET_BIT(rfid_msg.data.rfid_status_2 ,2 );
        rfid_parse.tunnel_enemy_4  = GET_BIT(rfid_msg.data.rfid_status_2 ,3 );
        rfid_parse.tunnel_enemy_5  = GET_BIT(rfid_msg.data.rfid_status_2 ,4 );
        rfid_parse.tunnel_enemy_6  = GET_BIT(rfid_msg.data.rfid_status_2 ,5 );

        static auto rfid_pub = create_publisher<rm_decision_interfaces::msg::RFIDParse>("rfid",10);
        // 发布 rfid 信息
        rfid_pub->publish(rfid_parse);
    }



    // 读取线程：定时从 Chiral 读取最新的目标跟踪数据，并发布 ROS 消息
    void chiralLoop()
    {
        rclcpp::Rate loop_rate(100); // 限制在 100Hz，足够绝大多数比赛场景
        while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed)) {
            if (!chiral_reader_) {
                std::this_thread::sleep_for(100ms);
                continue;
            }

            // 尝试读取新数据
            if (auto data = chiral_reader_->read_new()) {
                // TF 查询频率分离：每 N 帧查询一次 TF，其他帧使用缓存
                bool should_query_tf = (++tf_query_counter_ >= TF_QUERY_INTERVAL);
                if (should_query_tf) {
                    tf_query_counter_ = 0;
                }
                publishTargetTracking(*data, should_query_tf);
            } else {
                std::this_thread::sleep_for(100ms);
            }
            loop_rate.sleep(); // 强制限制循环频率，防止空转烧 CPU
        }
    }

    void publishTargetTracking(const talos::chrial::TalosData& talos_data, bool query_tf = true)
    {
        auto now = this->now();

        // 1. 发布目标跟踪状态
        armor_interfaces::msg::Target target_msg;
        target_msg.header.stamp = now;
        target_msg.header.frame_id = "map"; // 目标位置是相对于 odom 的绝对坐标

        // 跟踪器状态
        switch (talos_data.state.status) {
            case talos::chrial::TrackerStatus::Idle:
                target_msg.tracking = false;
                target_msg.tracking_status = 0; // Idle
                // Reset confidence when idle
                last_known_target_.valid = false;
                last_known_target_.confidence = 0.0;
                break;
            case talos::chrial::TrackerStatus::Detecting:
                target_msg.tracking = false;
                target_msg.tracking_status = 1; // Detecting
                // Decay confidence during detecting (target not locked yet)
                if (last_known_target_.valid) {
                    double time_since_last = (now - last_known_target_.last_update_time).seconds();
                    last_known_target_.confidence = calculateDecayedConfidence(last_known_target_.confidence, time_since_last);
                }
                target_msg.confidence = last_known_target_.confidence;
                break;
            case talos::chrial::TrackerStatus::Tracking:
                target_msg.tracking = true;
                target_msg.tracking_status = 2; // Tracking
                target_msg.confidence = 1.0;  // Full confidence when tracking
                break;
            case talos::chrial::TrackerStatus::TempLost:
                target_msg.tracking = false;
                target_msg.tracking_status = 3; // TempLost
                // Decay confidence during temporary loss using exponential decay
                if (last_known_target_.valid) {
                    double time_since_last = (now - last_known_target_.last_update_time).seconds();
                    last_known_target_.confidence = calculateDecayedConfidence(last_known_target_.confidence, time_since_last);
                    target_msg.confidence = last_known_target_.confidence;
                } else {
                    target_msg.confidence = 0.0;
                }
                break;
        }

        // 2. 计算敌方机器人在 odom 坐标系下的位置
        if (talos_data.state.status == talos::chrial::TrackerStatus::Tracking) {
            double enemy_x, enemy_y, enemy_z;
            double enemy_vx = 0, enemy_vy = 0, enemy_vz = 0;

            if (talos_data.state_kind == talos::chrial::TargetStateKind::Robot) {
                enemy_x = talos_data.state.robot.position.x;
                enemy_y = talos_data.state.robot.position.y;
                enemy_z = 0.0;
                enemy_vx = talos_data.state.robot.velocity.x;
                enemy_vy = talos_data.state.robot.velocity.y;
                enemy_vz = talos_data.state.robot.velocity.z;
            } else if (talos_data.state_kind == talos::chrial::TargetStateKind::Outpost) {
                enemy_x = talos_data.state.outpost.position.x;
                enemy_y = talos_data.state.outpost.position.y;
                enemy_z = 0.0;
                enemy_vx = talos_data.state.outpost.velocity.x;
                enemy_vy = talos_data.state.outpost.velocity.y;
                enemy_vz = talos_data.state.outpost.velocity.z;
            } else {
                // 无效的目标类型，发布空消息
                target_tracking_pub_->publish(target_msg);
                return;
            }

            // 坐标变换: gimbal_yaw -> gimbal_big -> map
            tf2::Vector3 enemy_gimbal_yaw(enemy_x, enemy_y, enemy_z);
            tf2::Vector3 velocity_gimbal_yaw(enemy_vx, enemy_vy, enemy_vz);

            // 1. gimbal_yaw -> gimbal_big
            // 注意：gimbal_link.rotation 表示从 gimbal_yaw 到 gimbal_big 的旋转
            tf2::Quaternion q_yaw_to_big(
                talos_data.gimbal_link.rotation.x,
                talos_data.gimbal_link.rotation.y,
                talos_data.gimbal_link.rotation.z,
                talos_data.gimbal_link.rotation.w
            );
            tf2::Vector3 enemy_gimbal_big = tf2::quatRotate(q_yaw_to_big, enemy_gimbal_yaw);

            // 提取 gimbal_yaw 相对 gimbal_big 的 yaw 角度，用于 gimbal_big 跟随
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_yaw_to_big).getRPY(roll, pitch, yaw);
            // 归一化到 [-π, π]
            while (yaw > M_PI) yaw -= 2 * M_PI;
            while (yaw < -M_PI) yaw += 2 * M_PI;

            // 更新 gimbal_big 跟随角度，加锁保护
            {
                std::lock_guard<std::mutex> lk(tx_mtx_);
                target_gimbal_big_yaw_angle_ = static_cast<float>(yaw);
                last_gimbal_angle_update_ = now;  // 记录更新时间
            }
            tf2::Vector3 velocity_gimbal_big = tf2::quatRotate(q_yaw_to_big, velocity_gimbal_yaw);

            // 2. gimbal_big -> map (使用 TF2，支持缓存以降低查询频率)
            geometry_msgs::msg::TransformStamped transform_map_to_big;

            if (query_tf) {
                // 查询新的 TF
                if (!tf_buffer_->canTransform("map", "gimbal_big", tf2::TimePointZero)) {
                    // 如果没有可用的 TF，尝试使用缓存
                    if (!has_cached_tf_) {
                        return;
                    }
                    transform_map_to_big = cached_map_to_gimbal_big_;
                } else {
                    try {
                        transform_map_to_big = tf_buffer_->lookupTransform("map", "gimbal_big", tf2::TimePointZero);
                        // 更新缓存
                        cached_map_to_gimbal_big_ = transform_map_to_big;
                        has_cached_tf_ = true;
                    } catch (const tf2::TransformException& ex) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "TF lookup map->gimbal_big failed: %s", ex.what());
                        if (!has_cached_tf_) {
                            target_tracking_pub_->publish(target_msg);
                            return;
                        }
                        transform_map_to_big = cached_map_to_gimbal_big_;
                    }
                }
            } else {
                // 使用缓存的 TF
                if (!has_cached_tf_) {
                    // 没有缓存，跳过本帧
                    return;
                }
                transform_map_to_big = cached_map_to_gimbal_big_;
            }

            try {

                tf2::Quaternion q_map_to_big(
                    transform_map_to_big.transform.rotation.x,
                    transform_map_to_big.transform.rotation.y,
                    transform_map_to_big.transform.rotation.z,
                    transform_map_to_big.transform.rotation.w
                );
                tf2::Vector3 t_map_to_big(
                    transform_map_to_big.transform.translation.x,
                    transform_map_to_big.transform.translation.y,
                    transform_map_to_big.transform.translation.z
                );

                // 变换位置和速度到 map 坐标系
                tf2::Vector3 enemy_map_rotated = tf2::quatRotate(q_map_to_big, enemy_gimbal_big);
                tf2::Vector3 enemy_map = enemy_map_rotated + t_map_to_big;
                tf2::Vector3 velocity_map = tf2::quatRotate(q_map_to_big, velocity_gimbal_big);

                // 将变换后的位置存入 Target 消息
                target_msg.position.x = enemy_map.x();
                target_msg.position.y = enemy_map.y();
                target_msg.position.z = enemy_map.z();
                target_msg.velocity.x = velocity_map.x();
                target_msg.velocity.y = velocity_map.y();
                target_msg.velocity.z = velocity_map.z();

                // 计算预测截断点（考虑底盘响应延迟）
                // 预测位置 = 当前位置 + 速度 × 延迟时间
                const double CHASSIS_RESPONSE_DELAY = 0.2;  // 底盘响应延迟（秒）
                double predicted_x = enemy_map.x() + velocity_map.x() * CHASSIS_RESPONSE_DELAY;
                double predicted_y = enemy_map.y() + velocity_map.y() * CHASSIS_RESPONSE_DELAY;
                double predicted_z = 0.0; // 机器人在平面上,毕竟要给导航用

                // Store last known target data for prediction
                last_known_target_.valid = true;
                last_known_target_.last_update_time = now;
                last_known_target_.position_x = enemy_map.x();
                last_known_target_.position_y = enemy_map.y();
                last_known_target_.position_z = enemy_map.z();
                last_known_target_.velocity_x = velocity_map.x();
                last_known_target_.velocity_y = velocity_map.y();
                last_known_target_.velocity_z = velocity_map.z();
                last_known_target_.predicted_x = predicted_x;
                last_known_target_.predicted_y = predicted_y;
                last_known_target_.predicted_z = predicted_z;
                last_known_target_.confidence = 1.0;

                RCLCPP_DEBUG(get_logger(), "Target predicted: pos=(%.2f, %.2f) -> pred=(%.2f, %.2f)",
                            enemy_map.x(), enemy_map.y(), predicted_x, predicted_y);

            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "TF transform failed: %s", ex.what());
                target_tracking_pub_->publish(target_msg);
                return;
            }
        } else if (talos_data.state.status == talos::chrial::TrackerStatus::TempLost &&
                   last_known_target_.valid &&
                   last_known_target_.confidence > min_confidence_threshold_) {
            // TempLost state with valid last known target and sufficient confidence
            // 从预测截断点开始，继续基于速度进行预测
            target_msg.position.x = last_known_target_.predicted_x;
            target_msg.position.y = last_known_target_.predicted_y;
            target_msg.position.z = last_known_target_.predicted_z;
            target_msg.velocity.x = last_known_target_.velocity_x;
            target_msg.velocity.y = last_known_target_.velocity_y;
            target_msg.velocity.z = last_known_target_.velocity_z;

            // 继续预测：从丢失时刻到现在的时间间隔内的运动
            double time_since_last = (now - last_known_target_.last_update_time).seconds();
            target_msg.position.x += last_known_target_.velocity_x * time_since_last;
            target_msg.position.y += last_known_target_.velocity_y * time_since_last;
            target_msg.position.z = 0.0;

            // Use last known target metadata
            target_msg.yaw = last_known_target_.yaw;
            target_msg.v_yaw = last_known_target_.v_yaw;
            target_msg.radius_1 = last_known_target_.radius_1;
            target_msg.radius_2 = last_known_target_.radius_2;
            target_msg.dz = last_known_target_.dz;
            target_msg.armors_num = last_known_target_.armors_num;
            target_msg.id = last_known_target_.id;

            RCLCPP_DEBUG(get_logger(), "Target TempLost: using predicted position (%.2f, %.2f) with confidence=%.2f",
                        target_msg.position.x, target_msg.position.y, last_known_target_.confidence);
        } else {
            // Non-tracking state or confidence too low, clear target data
            target_msg.position.x = 0;
            target_msg.position.y = 0;
            target_msg.position.z = 0;
            target_msg.velocity.x = 0;
            target_msg.velocity.y = 0;
            target_msg.velocity.z = 0;
            target_msg.confidence = 0.0;
        }

        // 3. 根据目标类型发布其他数据（位置和速度已通过坐标变换设置）
        if (talos_data.state_kind == talos::chrial::TargetStateKind::Robot) {
            // 机器人目标
            target_msg.yaw = talos_data.state.robot.yaw;
            target_msg.v_yaw = talos_data.state.robot.v_yaw;

            target_msg.radius_1 = talos_data.state.robot.radius0;
            target_msg.radius_2 = talos_data.state.robot.radius1;
            target_msg.dz = talos_data.state.robot.z1;

            target_msg.armors_num = talos_data.state.robot.armor_num;

            // 设置目标 ID
            std::ostringstream oss;
            switch (talos_data.state.name) {
                case talos::chrial::ArmorName::Sentry:
                    oss << "sentry";
                    break;
                case talos::chrial::ArmorName::One:
                    oss << "hero";
                    break;
                case talos::chrial::ArmorName::Two:
                    oss << "engineer";
                    break;
                case talos::chrial::ArmorName::Three:
                    oss << "standard_3";
                    break;
                case talos::chrial::ArmorName::Four:
                    oss << "standard_4";
                    break;
                case talos::chrial::ArmorName::Five:
                    oss << "standard_5";
                    break;
                case talos::chrial::ArmorName::Outpost:
                    oss << "outpost";
                    break;
                case talos::chrial::ArmorName::Base:
                    oss << "base";
                    break;
                default:
                    oss << "unknown";
                    break;
            }
            target_msg.id = oss.str();

            // Store metadata for prediction when tracking
            if (talos_data.state.status == talos::chrial::TrackerStatus::Tracking) {
                last_known_target_.yaw = target_msg.yaw;
                last_known_target_.v_yaw = target_msg.v_yaw;
                last_known_target_.radius_1 = target_msg.radius_1;
                last_known_target_.radius_2 = target_msg.radius_2;
                last_known_target_.dz = target_msg.dz;
                last_known_target_.armors_num = target_msg.armors_num;
                last_known_target_.id = target_msg.id;
                last_known_target_.target_kind = talos::chrial::TargetStateKind::Robot;
            }

        } else if (talos_data.state_kind == talos::chrial::TargetStateKind::Outpost) {
            // 前哨站目标
            target_msg.yaw = talos_data.state.outpost.yaw;
            target_msg.v_yaw = talos_data.state.outpost.v_yaw;

            target_msg.id = "outpost";
            target_msg.armors_num = 3; // 暂定为3，以后再改

            // Store metadata for prediction when tracking
            if (talos_data.state.status == talos::chrial::TrackerStatus::Tracking) {
                last_known_target_.yaw = target_msg.yaw;
                last_known_target_.v_yaw = target_msg.v_yaw;
                last_known_target_.radius_1 = 0;  // Outpost doesn't have these
                last_known_target_.radius_2 = 0;
                last_known_target_.dz = 0;
                last_known_target_.armors_num = target_msg.armors_num;
                last_known_target_.id = target_msg.id;
                last_known_target_.target_kind = talos::chrial::TargetStateKind::Outpost;
            }
        }

        target_tracking_pub_->publish(target_msg);

        /*  // 我在target消息里已经发布了跟踪状态了，这个字符串消息就没必要了，并且发布过多的消息可能会增加系统负担，
        // 况且Chiral的跟踪状态已经很清晰了，直接在Target消息里用一个字段表示就好了，不需要额外发布一个字符串消息来描述状态了
        // 我还要在rm_behavior_tree里订阅这个状态字符串来控制行为树的切换，如果再发布一个字符串消息的话就显得冗余了，
        // 直接在Target消息里用一个字段表示状态就好了，这样也更清晰明了，不需要额外维护一个字符串消息的发布和订阅了
        // 2. 发布跟踪器状态字符串
        std_msgs::msg::String status_msg;
        switch (talos_data.state.status) {
            case talos::chrial::TrackerStatus::Idle:
                status_msg.data = "Idle";
                break;
            case talos::chrial::TrackerStatus::Detecting:
                status_msg.data = "Detecting";
                break;
            case talos::chrial::TrackerStatus::Tracking:
                status_msg.data = "Tracking";
                break;
            case talos::chrial::TrackerStatus::TempLost:
                status_msg.data = "TempLost";
                break;
        }
        tracker_status_pub_->publish(status_msg);
        */

        // 注释：敌方位置已在函数开头通过坐标变换存入 Target 消息的 position 字段
        // 直接使用 Chiral 提供的位姿数据发布 进行位姿变换，省去发布多个 TF 的麻烦，况且 Chiral 已经在内部做了融合和滤波，直接使用它的结果会更稳定可靠
        

        // 4. 发布 TF 变换 (gimbal_link)
        /*
        geometry_msgs::msg::TransformStamped gimbal_tf;
        gimbal_tf.header.stamp = now;
        gimbal_tf.header.frame_id = "gimbal_big";
        gimbal_tf.child_frame_id = "gimbal_yaw";
        
        gimbal_tf.transform.translation.x = talos_data.gimbal_link.translation.x;
        gimbal_tf.transform.translation.y = talos_data.gimbal_link.translation.y;
        gimbal_tf.transform.translation.z = talos_data.gimbal_link.translation.z;

        gimbal_tf.transform.rotation.x = talos_data.gimbal_link.rotation.x;
        gimbal_tf.transform.rotation.y = talos_data.gimbal_link.rotation.y;
        gimbal_tf.transform.rotation.z = talos_data.gimbal_link.rotation.z;
        gimbal_tf.transform.rotation.w = talos_data.gimbal_link.rotation.w;

        tf_broadcaster_->sendTransform(gimbal_tf);
        */
        


        /*
        // 4. 发布 TF 变换 (muzzle_link)
        geometry_msgs::msg::TransformStamped muzzle_tf;
        muzzle_tf.header.stamp = now;
        muzzle_tf.header.frame_id = "gimbal_yaw";
        muzzle_tf.child_frame_id = "muzzle_link";

        muzzle_tf.transform.translation.x = talos_data.muzzle_link.translation.x;
        muzzle_tf.transform.translation.y = talos_data.muzzle_link.translation.y;
        muzzle_tf.transform.translation.z = talos_data.muzzle_link.translation.z;

        muzzle_tf.transform.rotation.x = talos_data.muzzle_link.rotation.x;
        muzzle_tf.transform.rotation.y = talos_data.muzzle_link.rotation.y;
        muzzle_tf.transform.rotation.z = talos_data.muzzle_link.rotation.z;
        muzzle_tf.transform.rotation.w = talos_data.muzzle_link.rotation.w;

        tf_broadcaster_->sendTransform(muzzle_tf);

        // 5. 发布 TF 变换 (camera_link)
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.stamp = now;
        camera_tf.header.frame_id = "gimbal_yaw";
        camera_tf.child_frame_id = "camera_link";

        camera_tf.transform.translation.x = talos_data.camera_link.translation.x;
        camera_tf.transform.translation.y = talos_data.camera_link.translation.y;
        camera_tf.transform.translation.z = talos_data.camera_link.translation.z;

        camera_tf.transform.rotation.x = talos_data.camera_link.rotation.x;
        camera_tf.transform.rotation.y = talos_data.camera_link.rotation.y;
        camera_tf.transform.rotation.z = talos_data.camera_link.rotation.z;
        camera_tf.transform.rotation.w = talos_data.camera_link.rotation.w;

        tf_broadcaster_->sendTransform(camera_tf);
        */
    }


    // 发送线程：定时将 current_cmd_state_ 和 current_robot_posture_state_ 发送给下位机
    void txLoop()
    {
        rclcpp::WallRate loop_rate { std::chrono::milliseconds(send_period_ms_) };
        while (rclcpp::ok() && !exit_.load()) {
            if (!is_port_ok_.load()) {
                loop_rate.sleep();
                continue;
            }

            // 发送机器人控制命令 (底盘速度 + 云台速度)
            {
                rm_sentry_pp::SendRobotCmdData pkt {};
                {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    rm_sentry_pp::fillHeader(pkt, rm_sentry_pp::ID_ROBOT_CMD);
                    pkt.frame_header.id = rm_sentry_pp::ID_ROBOT_CMD;
                    pkt.time_stamp = nowMs();
                    pkt.data.speed_vector = current_cmd_state_.data.speed_vector;

                    // gimbal_big 控制模式：角度或速度，互斥
                    // 检查角度数据是否超时
                    
                    pkt.data.gimbal_big.yaw_angle = gimbal_big_yaw_angle_; // 持续发布最新的角度值，无论是否超时    
                    pkt.data.gimbal_big.yaw_vel = current_cmd_state_.data.gimbal_big.yaw_vel;
                    
                    pkt.eof = rm_sentry_pp::HeaderFrame::EoF();
                }

                auto bytes = rm_sentry_pp::toVector(pkt);
                std::lock_guard<std::mutex> lk(port_mtx_);
                if (!sp_.writeAll(bytes.data(), bytes.size())) {
                    is_port_ok_.store(false);
                }
            }

            // 发送机器人姿态命令
            {
                rm_sentry_pp::SendRobotPostureData pkt {};
                {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    rm_sentry_pp::fillHeader(pkt, rm_sentry_pp::ID_ROBOT_POSTURE);
                    pkt.frame_header.id = rm_sentry_pp::ID_ROBOT_POSTURE;
                    pkt.time_stamp = nowMs();
                    pkt.data.posture = current_robot_posture_state_.data.posture;
                    pkt.data.follow_gimbal_big = follow_gimbal_big_;
                    pkt.eof = rm_sentry_pp::HeaderFrame::EoF();
                }

                auto bytes = rm_sentry_pp::toVector(pkt);
                std::lock_guard<std::mutex> lk(port_mtx_);
                if (!sp_.writeAll(bytes.data(), bytes.size())) {
                    is_port_ok_.store(false);
                }
            }

            loop_rate.sleep();
        }
    }

private:
    // params
    std::string port_;
    int baud_ { 115200 };
    std::string imu_frame_;
    std::string imu_parent_frame_ { "gimbal_big" };
    std::string cmd_vel_chassis_topic_;
    std::string robot_control_topic_;
    std::string imu_topic_;
    std::string set_posture_service_name_;
    std::string robot_info_topic_;
    int send_period_ms_ { 5 };
    bool enable_dtr_rts_ { true };
    float target_spin_vel_ = 0.0f;
    int gimbal_angle_timeout_ms_ { 300 };  // gimbal 角度数据超时时间 (ms)
    std::string gimbal_follow_path_topic_;
    double gimbal_follow_lookahead_ { 1.5 };

    // ros
    rclcpp::Time node_start_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<armor_interfaces::msg::Target>::SharedPtr target_tracking_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gimbal_yaw_marker_pub_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotControl>::SharedPtr robot_control_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Service<rm_decision_interfaces::srv::SetSentryPosture>::SharedPtr set_posture_service_;

    // serial
    SerialPort sp_;
    std::mutex port_mtx_;
    std::atomic<bool> is_port_ok_ { false };

    // threads
    std::atomic<bool> exit_ { false };
    std::thread protect_thread_;
    std::thread rx_thread_;
    std::thread tx_thread_;
    std::thread chiral_thread_;

    // tx data
    std::mutex tx_mtx_;
    //rm_sentry_pp::SendRobotCmdData tx_pkt_ {};
    rm_decision_interfaces::msg::RobotControl last_robot_control_cmd_;
    rm_sentry_pp::SendRobotCmdData current_cmd_state_; // 存储最新的底盘和云台期望值
    rm_sentry_pp::SendRobotPostureData current_robot_posture_state_; // 存储最新的机器人姿态信息

    // Gimbal angle follow data (protected by tx_mtx_)
    float target_gimbal_big_yaw_angle_ = 0.0f;    // 目标大云台角度（用于跟随 gimbal_yaw）
    rclcpp::Time last_gimbal_angle_update_;       // 最后更新时间戳，用于超时检测
    float gimbal_big_yaw_angle_ = 0.0f;           // 获得持续发布的角度
    float follow_gimbal_big_ = 0.0f;              // 底盘跟随
    float gimbal_yaw_ = 0.0f;                     // gimbal_yaw 机械角

    // Gimbal path follow - high frequency resampling
    nav_msgs::msg::Path cached_path_;
    std::mutex path_mtx_;
    rclcpp::TimerBase::SharedPtr gimbal_path_timer_;
    rclcpp::Time last_path_time_;
    int gimbal_path_timeout_ms_ { 1000 };

    // IMU drift correction (protected by tx_mtx_)
    double imu_yaw_offset_ = 0.0;                 // IMU yaw 偏移量（用于修正漂移）
    bool is_calibrating_imu_ = false;             // 是否正在校准 IMU
    rclcpp::Time last_imu_calibration_time_;      // 上次 IMU 校准时间
    static constexpr double IMU_CALIBRATION_THRESHOLD = 0.05;  // 云台回中阈值（弧度）
    static constexpr double IMU_CALIBRATION_INTERVAL = 2.0;    // 校准间隔（秒）

    /*
    rm_sentry_pp::ReceiveRobotInfoData current_robot_info_state_; // 存储最新的机器人信息
    rm_sentry_pp::ReceiveGameStatusData current_game_status_state_; // 存储最新的比赛状态信息
    rm_sentry_pp::ReceiveAllRobotHpData current_all_robot_hp_state_; // 存储最新的所有机器人血量信息
    rm_sentry_pp::ReceiveRobotLocation current_robot_location_state_; // 存储最新的机器人位置信息
    */


    bool tx_pending_ { false };
    bool tx_posture_pending_ { false };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // chiral
    std::unique_ptr<talos::chiral::ipc::TalosDataReader> chiral_reader_;

    // Target lost prediction - store last known target data
    struct LastKnownTarget {
        bool valid = false;
        rclcpp::Time last_update_time;
        double position_x = 0, position_y = 0, position_z = 0;
        double velocity_x = 0, velocity_y = 0, velocity_z = 0;
        double yaw = 0, v_yaw = 0;
        double radius_1 = 0, radius_2 = 0, dz = 0;
        int32_t armors_num = 0;
        std::string id;
        talos::chrial::TargetStateKind target_kind = talos::chrial::TargetStateKind::Robot;
        double confidence = 1.0;  // Confidence starts at 1.0 when tracking

        // 预测截断点（用于底盘追击）
        double predicted_x = 0, predicted_y = 0, predicted_z = 0;
    } last_known_target_;

    // TF query frequency control
    int tf_query_counter_ = 0;
    static constexpr int TF_QUERY_INTERVAL = 10;  // 每 10 帧（10Hz）查询一次 TF
    geometry_msgs::msg::TransformStamped cached_map_to_gimbal_big_;  // 缓存的 TF 变换
    bool has_cached_tf_ = false;

    // Confidence decay parameters
    double confidence_decay_lambda_ = 0.5;  // Decay rate (lambda) for exponential decay
    double min_confidence_threshold_ = 0.3;  // Minimum confidence before marking as lost

    // Helper function for exponential confidence decay
    double calculateDecayedConfidence(double current_confidence, double dt) {
        // Exponential decay: Confidence_t = Confidence_{t-1} * e^(-lambda * dt)
        return std::max(0.0, current_confidence * std::exp(-confidence_decay_lambda_ * dt));
    }
};

} // namespace rm_sentry_pp_nocrc_serial

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rm_sentry_pp_nocrc_serial::Node>(rclcpp::NodeOptions {}));
    rclcpp::shutdown();
    return 0;
}
