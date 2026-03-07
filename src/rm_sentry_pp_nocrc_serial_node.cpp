#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_decision_interfaces/msg/detail/friend_location__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <auto_aim_interfaces/msg/target.hpp>
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

        // 初始化 TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

        // Chiral 目标跟踪数据发布者
        target_tracking_pub_ = create_publisher<auto_aim_interfaces::msg::Target>("target_tracking", 10);
        // tracker_status_pub_ = create_publisher<std_msgs::msg::String>("tracker_status", 10);
        // 敌方位置在 base_footprint 坐标系下的发布者
        enemy_position_base_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("enemy_position_base", 10);

        cmd_vel_chassis_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_chassis_topic_, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { onCmd(*msg); });

        robot_control_sub_ = create_subscription<rm_decision_interfaces::msg::RobotControl>(
            robot_control_topic_, 10,
            [this](const rm_decision_interfaces::msg::RobotControl::SharedPtr msg) { onRobotControl(*msg); });

        // 创建服务服务器替代话题订阅
        set_posture_service_ = create_service<rm_decision_interfaces::srv::SetSentryPosture>(
            set_posture_service_name_,
            [this](const std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Request> request,
                   std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Response> response) {
                handleSetSentryPosture(request, response);
            });
        RCLCPP_INFO(get_logger(), "Service server created: %s", set_posture_service_name_.c_str());

        node_start_ = this->now();

        // 初始化 Chiral 读取器
        auto chiral_reader = talos::chiral::ipc::TalosDataReader::open();
        if (chiral_reader) {
            chiral_reader_ = std::make_unique<talos::chiral::ipc::TalosDataReader>(std::move(*chiral_reader));
            RCLCPP_INFO(get_logger(), "Chiral reader initialized successfully");
        } else {
            RCLCPP_WARN(get_logger(), "Failed to initialize chiral reader: %d", static_cast<int>(chiral_reader.error()));
        }

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
        current_cmd_state_.data.speed_vector.wz = msg.angular.z;
        tx_pending_ = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Twist in: vx=%.3f vy=%.3f wz=%.3f | stored: vx=%d vy=%d wz=%d",
            msg.linear.x, msg.linear.y, msg.angular.z,
            (int)current_cmd_state_.data.speed_vector.vx,
            (int)current_cmd_state_.data.speed_vector.vy,
            (int)current_cmd_state_.data.speed_vector.wz);
    }

    void onRobotControl(const rm_decision_interfaces::msg::RobotControl& msg)
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        current_cmd_state_.data.gimbal_big.yaw_vel = msg.gimbal_big_yaw_vel;
        tx_pending_ = true;
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

        tf2::Quaternion q;
        // 假设 imu_data 里的顺序是 Roll, Pitch, Yaw
        q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
        imu.orientation = tf2::toMsg(q);

        imu.angular_velocity.x = imu_data.data.roll_vel;
        imu.angular_velocity.y = imu_data.data.pitch_vel;
        imu.angular_velocity.z = imu_data.data.yaw_vel;

        imu_pub_->publish(imu);

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

        tf_broadcaster_->sendTransform(t);
    }

    void publishRobotInfo(const rm_sentry_pp::ReceiveRobotInfoData& robot_info_data)
    {
        auto now = this->now();

        rm_decision_interfaces::msg::SentryPostureStatus posture_msg;
        posture_msg.reported_posture = robot_info_data.data.posture;


        // 发布机器人姿态状态
        // 这里假设你有一个名为 "sentry_posture_status" 的话题
        static auto posture_pub = create_publisher<rm_decision_interfaces::msg::SentryPostureStatus>("sentry_posture_status", 10);
        posture_pub->publish(posture_msg);

        rm_decision_interfaces::msg::RobotStatus robot_status_msg;
        robot_status_msg.robot_id = robot_info_data.data.id;
        robot_status_msg.team_color = robot_info_data.data.color;
        robot_status_msg.is_attacked = robot_info_data.data.attacked;
        robot_status_msg.current_hp = robot_info_data.data.hp;
        
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

    // 读取线程：定时从 Chiral 读取最新的目标跟踪数据，并发布 ROS 消息
    void chiralLoop()
    {
        while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed)) {
            if (!chiral_reader_) {
                std::this_thread::sleep_for(100ms);
                continue;
            }

            // 尝试读取新数据
            if (auto data = chiral_reader_->read_new()) {
                publishTargetTracking(*data);
            } else {
                std::this_thread::sleep_for(1ms);
            }
        }
    }

    void publishTargetTracking(const talos::chrial::TalosData& talos_data)
    {
        auto now = this->now();

        // 1. 发布目标跟踪状态
        auto_aim_interfaces::msg::Target target_msg;
        target_msg.header.stamp = now;
        target_msg.header.frame_id = "odom";

        // 跟踪器状态
        switch (talos_data.state.status) {
            case talos::chrial::TrackerStatus::Idle:
                target_msg.tracking = false;
                target_msg.tracking_status = 0; // Idle
                break;
            case talos::chrial::TrackerStatus::Detecting:
                target_msg.tracking = false;
                target_msg.tracking_status = 1; // Detecting
                break;
            case talos::chrial::TrackerStatus::Tracking:
                target_msg.tracking = true;
                target_msg.tracking_status = 2; // Tracking
                break;
            case talos::chrial::TrackerStatus::TempLost:
                target_msg.tracking = false;
                target_msg.tracking_status = 3; // TempLost
                break;
        }

        // 根据目标类型发布不同的数据
        if (talos_data.state_kind == talos::chrial::TargetStateKind::Robot) {
            // 机器人目标
            target_msg.position.x = talos_data.state.robot.position.x;
            target_msg.position.y = talos_data.state.robot.position.y;
            target_msg.position.z = talos_data.state.robot.position.z;

            target_msg.velocity.x = talos_data.state.robot.velocity.x;
            target_msg.velocity.y = talos_data.state.robot.velocity.y;
            target_msg.velocity.z = talos_data.state.robot.velocity.z;

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

        } else if (talos_data.state_kind == talos::chrial::TargetStateKind::Outpost) {
            // 前哨站目标
            target_msg.position.x = talos_data.state.outpost.position.x;
            target_msg.position.y = talos_data.state.outpost.position.y;
            target_msg.position.z = talos_data.state.outpost.position.z;

            target_msg.velocity.x = talos_data.state.outpost.velocity.x;
            target_msg.velocity.y = talos_data.state.outpost.velocity.y;
            target_msg.velocity.z = talos_data.state.outpost.velocity.z;

            target_msg.yaw = talos_data.state.outpost.yaw;
            target_msg.v_yaw = talos_data.state.outpost.v_yaw;

            target_msg.id = "outpost";
            target_msg.armors_num = 3;
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

        // 3. 计算并发布敌方机器人在 base_footprint 坐标系下的位置
        if (talos_data.state.status == talos::chrial::TrackerStatus::Tracking) {
            publishEnemyPositionInBaseFrame(talos_data, now);
        }

        // 直接使用 Chiral 提供的位姿数据发布 进行位姿变换，省去发布多个 TF 的麻烦，况且 Chiral 已经在内部做了融合和滤波，直接使用它的结果会更稳定可靠
        /*

        // 4. 发布 TF 变换 (gimbal_link)
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

    void publishEnemyPositionInBaseFrame(const talos::chrial::TalosData& talos_data, const rclcpp::Time& stamp)
    {
        try {
            // 获取敌方机器人在 gimbal_yaw 坐标系下的位置
            double enemy_x, enemy_y, enemy_z;

            if (talos_data.state_kind == talos::chrial::TargetStateKind::Robot) {
                enemy_x = talos_data.state.robot.position.x;
                enemy_y = talos_data.state.robot.position.y;
                enemy_z = talos_data.state.robot.position.z;
            } else if (talos_data.state_kind == talos::chrial::TargetStateKind::Outpost) {
                enemy_x = talos_data.state.outpost.position.x;
                enemy_y = talos_data.state.outpost.position.y;
                enemy_z = talos_data.state.outpost.position.z;
            } else {
                return; // 无效的目标类型
            }

            // 坐标变换: gimbal_yaw -> gimbal_big -> odom
            // 变换链: odom -> gimbal_big ->[yaw旋转]-> gimbal_yaw

            // 1. 使用 gimbal_link 的旋转部分: gimbal_yaw -> gimbal_big
            // gimbal_link 描述 gimbal_big -> gimbal_yaw 的变换
            // 主要包含 yaw 旋转（小云台相对于大云台的水平转动）
            tf2::Quaternion q_big_to_yaw(
                talos_data.gimbal_link.rotation.x,
                talos_data.gimbal_link.rotation.y,
                talos_data.gimbal_link.rotation.z,
                talos_data.gimbal_link.rotation.w
            );

            // 敌方位置在 gimbal_yaw 坐标系下
            tf2::Vector3 enemy_gimbal_yaw(enemy_x, enemy_y, enemy_z);

            // 逆变换: gimbal_yaw -> gimbal_big (忽略平移，只考虑旋转)
            // P_big = R^T * P_yaw
            tf2::Quaternion q_yaw_to_big = q_big_to_yaw.inverse();
            tf2::Vector3 enemy_gimbal_big = tf2::quatRotate(q_yaw_to_big, enemy_gimbal_yaw);

            // 2. 使用 TF2 获取 map -> gimbal_big 的变换
            // 使用map测试一下
            geometry_msgs::msg::TransformStamped transform_odom_to_big;
            try {
                transform_odom_to_big = tf_buffer_->lookupTransform(
                    "map", "gimbal_big",
                    stamp,
                    rclcpp::Duration::from_seconds(0.10)); // 100ms 超时
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "TF lookup map->gimbal_big failed: %s", ex.what());
                return;
            }

            // 从 TF2 提取变换
            tf2::Quaternion q_odom_to_big(
                transform_odom_to_big.transform.rotation.x,
                transform_odom_to_big.transform.rotation.y,
                transform_odom_to_big.transform.rotation.z,
                transform_odom_to_big.transform.rotation.w
            );

            tf2::Vector3 t_odom_to_big(
                transform_odom_to_big.transform.translation.x,
                transform_odom_to_big.transform.translation.y,
                transform_odom_to_big.transform.translation.z
            );

            // 3. 变换: gimbal_big -> odom
            // P_odom = R * P_big + t
            tf2::Vector3 enemy_odom_rotated = tf2::quatRotate(q_odom_to_big, enemy_gimbal_big);
            tf2::Vector3 enemy_odom = enemy_odom_rotated + t_odom_to_big;

            // 发布敌方位置
            geometry_msgs::msg::PointStamped enemy_msg;
            enemy_msg.header.stamp = stamp;
            enemy_msg.header.frame_id = "odom";
            enemy_msg.point.x = enemy_odom.x();
            enemy_msg.point.y = enemy_odom.y();
            enemy_msg.point.z = enemy_odom.z();

            enemy_position_base_pub_->publish(enemy_msg);

            RCLCPP_DEBUG(get_logger(), "Enemy position in odom: [%.3f, %.3f, %.3f]",
                enemy_msg.point.x, enemy_msg.point.y, enemy_msg.point.z);

        } catch (const std::exception& ex) {
            RCLCPP_ERROR(get_logger(), "Error transforming enemy position: %s", ex.what());
        }
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
                    pkt.data = current_cmd_state_.data;
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
                    pkt.data = current_robot_posture_state_.data;
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

    // ros
    rclcpp::Time node_start_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_tracking_pub_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotControl>::SharedPtr robot_control_sub_;
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
    rm_sentry_pp::SendRobotCmdData tx_pkt_ {};
    rm_decision_interfaces::msg::RobotControl last_robot_control_cmd_;
    rm_sentry_pp::SendRobotCmdData current_cmd_state_; // 存储最新的底盘和云台期望值
    rm_sentry_pp::SendRobotPostureData current_robot_posture_state_; // 存储最新的机器人姿态信息

    rm_sentry_pp::ReceiveRobotInfoData current_robot_info_state_; // 存储最新的机器人信息
    rm_sentry_pp::ReceiveGameStatusData current_game_status_state_; // 存储最新的比赛状态信息
    rm_sentry_pp::ReceiveAllRobotHpData current_all_robot_hp_state_; // 存储最新的所有机器人血量信息
    rm_sentry_pp::ReceiveRobotLocation current_robot_location_state_; // 存储最新的机器人位置信息


    bool tx_pending_ { false };
    bool tx_posture_pending_ { false };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // chiral
    std::unique_ptr<talos::chiral::ipc::TalosDataReader> chiral_reader_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr enemy_position_base_pub_;
};

} // namespace rm_sentry_pp_nocrc_serial

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rm_sentry_pp_nocrc_serial::Node>(rclcpp::NodeOptions {}));
    rclcpp::shutdown();
    return 0;
}
