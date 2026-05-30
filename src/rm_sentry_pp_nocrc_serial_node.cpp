#include "chiral/chiral_endpoint.hpp"
#include "chiral/navigation.hpp"
#include "rm_sentry_pp_nocrc_serial/node.hpp"
#include "rm_sentry_pp_nocrc_serial/packet.hpp"
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/enemy_forbidden_area__struct.hpp>
#include <rm_decision_interfaces/msg/detail/enemy_location__struct.hpp>
#include <rm_decision_interfaces/msg/detail/self_robot_hp__struct.hpp>

using namespace std::chrono_literals;

namespace rm_sentry_pp_nocrc_serial {

Node::Node(const rclcpp::NodeOptions& options)
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
    posture_confirm_timeout_ms_ = declare_parameter<int>("posture_confirm_timeout_ms", 500);
    gimbal_follow_path_topic_ = declare_parameter<std::string>("gimbal_follow_path_topic", "plan");
    gimbal_follow_lookahead_ = declare_parameter<double>("gimbal_follow_lookahead", 1.5);
    gimbal_lookahead_base_ = declare_parameter<double>("gimbal_lookahead_base", 0.8);
    gimbal_lookahead_k_ = declare_parameter<double>("gimbal_lookahead_k", 0.4);
    gimbal_yaw_smooth_alpha_ = declare_parameter<double>("gimbal_yaw_smooth_alpha", 0.3);
    robot_area_name_ = declare_parameter<std::string>("robot_area_name", "bumpy_area");

    RCLCPP_INFO(get_logger(), "Gimbal follow: base=%.2f, k=%.2f, alpha=%.2f",
                gimbal_lookahead_base_, gimbal_lookahead_k_, gimbal_yaw_smooth_alpha_);

    // Odometry 参数
    odom_topic_ = declare_parameter<std::string>("odom_topic", "lidar_odometry");
    relocalization_mode_ = declare_parameter<bool>("relocalization_mode", false);
    odom_timeout_ms_ = declare_parameter<int>("odom_timeout_ms", 500);
    RCLCPP_INFO(get_logger(), "Odometry: topic=%s, relocalization=%d",
                odom_topic_.c_str(), relocalization_mode_);

    // 初始化 TF2 (仅重定位模式需要查询 map→odom)
    if (relocalization_mode_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

    // Chiral 目标跟踪数据发布者
    target_tracking_pub_ = create_publisher<armor_interfaces::msg::Target>("target_tracking", 10);
    gimbal_yaw_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("expected_gimbal_yaw", 10);
    lookahead_point_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_point_marker", 10);
    enemy_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("enemy_marker", 10);

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

    // Odometry 订阅
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(*msg); });

    // map→odom 定时查询（仅重定位模式）
    if (relocalization_mode_) {
        map_odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { updateMapToOdom(); });
    }

    enemy_forbidden_area_sub = create_subscription<rm_decision_interfaces::msg::EnemyForbiddenArea>("enemy_in_forbidden_area",10,
    [this](const rm_decision_interfaces::msg::EnemyForbiddenArea::SharedPtr msg) { updateEnemyForbiddenArea(*msg); });

    robot_area_status_sub = create_subscription<rm_decision_interfaces::msg::RobotAreaStatus>("robot_area_status",10,
    [this](const rm_decision_interfaces::msg::RobotAreaStatus::SharedPtr msg) { onRobotAreaStatus(*msg); });

    // 机器人姿态状态
    posture_pub_ = create_publisher<rm_decision_interfaces::msg::SentryPostureStatus>("sentry_posture_status", 10);

    // 机器人状态消息
    robot_status_pub_ = create_publisher<rm_decision_interfaces::msg::RobotStatus>("robot_status", 10);

    // 比赛状态信息
    game_status_pub_ = create_publisher<rm_decision_interfaces::msg::GameStatus>("game_status", 10);

    // 己方机器人血量消息  因为敌方机器人血量信息暂时没有，所以这个消息里先只发布己方机器人血量
    all_robot_hp_pub_ = create_publisher<rm_decision_interfaces::msg::SelfRobotHP>("self_robot_hp", 10); 

    // 是否开始保存地图消息  （是否保存odin1地图）
    start_save_map_pub_ = create_publisher<std_msgs::msg::Bool>("start_save_map",10);

    // 机器人位置信息消息
    robot_location_pub_ = create_publisher<rm_decision_interfaces::msg::FriendLocation>("robot_location", 10);

    // rfid 信息
    rfid_pub_ = create_publisher<rm_decision_interfaces::msg::RFIDParse>("rfid",10);

    // 敌方机器人位置
    enemy_location_pub_ = create_publisher<rm_decision_interfaces::msg::EnemyLocation>("enemy_location",10);
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
    last_gimbal_angle_update_ = node_start_;
    last_path_time_ = node_start_;
    last_drift_update_ = node_start_;
    last_odom_time_ = node_start_;

    // 初始化 Chiral 读取器
    auto chiral_reader = talos::chiral::navigation::NavigationEndpoint::create();
    if (chiral_reader) {
        chiral_reader_ = std::move(chiral_reader.value());
        RCLCPP_INFO(get_logger(), "Chiral reader initialized successfully");
    } else {
        RCLCPP_WARN(get_logger(), "Failed to initialize chiral reader: %d", static_cast<int>(chiral_reader.error()));
    }
    auto nd =talos::chiral::navigation::NavigationData::create();
    nd.set_invincible(talos::chiral::navigation::ArmorName::Sentry, true);
    chiral_reader_->write(nd);

    // Gimbal 路径跟随高频重采样 timer
    gimbal_path_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz
        [this]() { updateGimbalFromCachedPath(); });

    // gimbal_big 漂移校正定时器（从串口线程分离，避免阻塞接收）
    drift_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),  // 20Hz
        [this]() { updateDriftCorrection(); });

    protect_thread_ = std::thread([this]() { protectLoop(); });
    rx_thread_ = std::thread([this]() { rxLoop(); });
    tx_thread_ = std::thread([this]() { txLoop(); });
    chiral_thread_ = std::thread([this]() { chiralLoop(); });

    RCLCPP_INFO(get_logger(), "rm_sentry_pp_nocrc_serial started.");
}

Node::~Node()
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

uint32_t Node::nowMs() const
{
    auto dt = (this->now() - node_start_).nanoseconds();
    return static_cast<uint32_t>(dt / 1000000ULL);
}

void Node::onCmd(const geometry_msgs::msg::Twist& msg)
{
    std::lock_guard<std::mutex> lk(tx_mtx_);
    current_cmd_state_.data.speed_vector.vx = msg.linear.x;
    current_cmd_state_.data.speed_vector.vy = msg.linear.y;
    current_cmd_state_.data.speed_vector.wz = target_spin_vel_;

    tx_pending_ = true;
}

void Node::onRobotControl(const rm_decision_interfaces::msg::RobotControl& msg)
{
    std::lock_guard<std::mutex> lk(tx_mtx_);
    current_cmd_state_.data.gimbal_big.yaw_vel = msg.gimbal_big_yaw_vel;
    target_spin_vel_ = msg.chassis_spin_vel;
    start_gimbal_big_spin_ = msg.start_gimbal_big_spin;
    // follow_gimbal_big_ = msg.follow_gimbal_big;
    // track_status_ = msg.track_status;
    // perception_status_ = msg.perception_status;
    tx_pending_ = true;
}

void Node::onPath(const nav_msgs::msg::Path& msg)
{
    if (msg.poses.empty()) return;
    std::lock_guard<std::mutex> lk(path_mtx_);
    cached_path_ = msg;
    last_path_time_ = this->now();
}

void Node::updateGimbalFromCachedPath()
{
    rclcpp::Time path_time;
    {
        std::lock_guard<std::mutex> lk(path_mtx_);
        path_time = last_path_time_;
    }
    if ((this->now() - path_time).seconds() * 1000.0 > gimbal_path_timeout_ms_) return;

    nav_msgs::msg::Path local_path;
    {
        std::lock_guard<std::mutex> lk(path_mtx_);
        if (cached_path_.poses.empty()) return;
        local_path = cached_path_;
    }

    double chassis_x, chassis_y, chassis_yaw;
    if (!getChassisPoseInMap(chassis_x, chassis_y, chassis_yaw)) return;

    // --- Speed-dependent lookahead ---
    double vx, vy;
    {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        vx = cached_odom_vx_;
        vy = cached_odom_vy_;
    }
    // Transform velocity to map frame for true speed
    double speed = std::hypot(
        vx * std::cos(chassis_yaw) - vy * std::sin(chassis_yaw),
        vx * std::sin(chassis_yaw) + vy * std::cos(chassis_yaw));
    double lookahead = gimbal_lookahead_base_ + gimbal_lookahead_k_ * speed;

    // --- Step 1: Find nearest path point (search from prev_nearest_idx_) ---
    const size_t N = local_path.poses.size();
    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    // Warm-start from previous nearest to handle forward motion efficiently
    size_t search_start = (prev_nearest_idx_ > 0 && prev_nearest_idx_ < N)
                              ? prev_nearest_idx_ - 1 : 0;
    for (size_t i = search_start; i < N; ++i) {
        double dx = local_path.poses[i].pose.position.x - chassis_x;
        double dy = local_path.poses[i].pose.position.y - chassis_y;
        double d = dx * dx + dy * dy;
        if (d < min_dist) {
            min_dist = d;
            nearest_idx = i;
        }
        // If distance starts growing and we've passed the minimum, stop early
        if (d > min_dist + 1.0 && i > nearest_idx + 1) break;
    }
    prev_nearest_idx_ = nearest_idx;

    // --- Step 2: Walk along path arc-length from nearest_idx ---
    // Find the point that is `lookahead` meters ahead on the path
    double accumulated = 0.0;
    size_t target_idx = nearest_idx;
    double target_x = local_path.poses[nearest_idx].pose.position.x;
    double target_y = local_path.poses[nearest_idx].pose.position.y;
    double target_yaw_in_map = 0.0;

    auto extractYaw = [](const geometry_msgs::msg::Quaternion& q) -> double {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double r, p, y;
        tf2::Matrix3x3(quat).getRPY(r, p, y);
        return y;
    };

    for (size_t i = nearest_idx; i + 1 < N; ++i) {
        double x0 = local_path.poses[i].pose.position.x;
        double y0 = local_path.poses[i].pose.position.y;
        double x1 = local_path.poses[i + 1].pose.position.x;
        double y1 = local_path.poses[i + 1].pose.position.y;
        double seg_len = std::hypot(x1 - x0, y1 - y0);

        if (accumulated + seg_len >= lookahead) {
            // Interpolate position within this segment
            double frac = (lookahead - accumulated) / seg_len;
            target_x = x0 + frac * (x1 - x0);
            target_y = y0 + frac * (y1 - y0);
            // Use only the forward point's orientation (avoid corrupted near-car yaw)
            target_yaw_in_map = extractYaw(local_path.poses[i + 1].pose.orientation);
            target_idx = i + 1;
            break;
        }
        accumulated += seg_len;
        target_idx = i + 1;
    }

    // If we ran out of path before reaching lookahead, use last point
    if (accumulated < lookahead && target_idx >= N - 1) {
        target_x = local_path.poses.back().pose.position.x;
        target_y = local_path.poses.back().pose.position.y;
        target_yaw_in_map = extractYaw(local_path.poses.back().pose.orientation); // 这里反而需要直接使用这个角度，全局规划器给出的方向是在 哨兵坐标系下的 ，可以直接用
    }


    // --- Step 4: Low-pass filter with angle wrapping ---
    {
        float prev = gimbal_yaw_filtered_;
        float diff = target_yaw_in_map - prev;
        while (diff > static_cast<float>(M_PI)) diff -= 2 * static_cast<float>(M_PI);
        while (diff < static_cast<float>(-M_PI)) diff += 2 * static_cast<float>(M_PI);
        float filtered = prev + static_cast<float>(gimbal_yaw_smooth_alpha_) * diff;

        std::lock_guard<std::mutex> lk(tx_mtx_);
        gimbal_yaw_filtered_ = filtered;
        gimbal_big_yaw_angle_ = std::atan2(std::sin(filtered), std::cos(filtered));
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,   // ms
            "角度 %f",
            gimbal_big_yaw_angle_
        );
        last_gimbal_angle_update_ = this->now();
    }

    // --- Visualization: lookahead point (green sphere in map frame) ---
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp = this->now();
        m.header.frame_id = "map";
        m.ns = "lookahead_point_marker";
        m.id = 1;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = target_x;
        m.pose.position.y = target_y;
        m.pose.position.z = 0.15;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.15;
        m.scale.y = 0.15;
        m.scale.z = 0.15;
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
        m.lifetime = rclcpp::Duration(0, 0); // 0.5s
        lookahead_point_marker_pub_->publish(m);
    }

    // --- Visualization: expected yaw (red arrow in gimbal_big frame) ---
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "gimbal_big";
        marker.ns = "expected_gimbal_yaw";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        tf2::Quaternion mq;
        mq.setRPY(0, 0, gimbal_yaw_filtered_);
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
}

void Node::updateDriftCorrection()
{
    auto now = this->now();

    // 需要归中参考和 odom 稳定后才开始校正
    if (!has_imu_centering_ref_ || !odom_stable_) return;

    double imu_raw_yaw;
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        imu_raw_yaw = std::atan2(std::sin(latest_imu_raw_yaw_), std::cos(latest_imu_raw_yaw_));
    }

    double odom_yaw;
    {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        if ((now - last_odom_time_).seconds() * 1000.0 > odom_timeout_ms_) return;
        odom_yaw = cached_odom_yaw_;
    }

    // 补偿量 = (IMU当前 - odom当前) - IMU归中时刻值
    double compensation = (imu_raw_yaw - odom_yaw) - (imu_at_centering_ - odom_at_centering_);
    while (compensation > M_PI) compensation -= 2 * M_PI;
    while (compensation < -M_PI) compensation += 2 * M_PI;

    std::lock_guard<std::mutex> lk(tx_mtx_);

    // 低通滤波
    double filtered = gimbal_big_drift_ + DRIFT_FILTER_ALPHA * (compensation - gimbal_big_drift_);

    // 更新漂移速率（用于 txLoop 发送间隔插值）
    double dt = (now - last_drift_update_).seconds();
    if (dt > 0.5) {
        gimbal_big_drift_rate_ = 0.0;
    } else if (dt > 0.001) {
        double rate_raw = (filtered - gimbal_big_drift_) / dt;
        rate_raw = std::clamp(rate_raw, -1.0, 1.0);
        gimbal_big_drift_rate_ = 0.8 * gimbal_big_drift_rate_ + 0.2 * rate_raw;
    }

    gimbal_big_drift_ = std::atan2(std::sin(filtered), std::cos(filtered));
    last_drift_update_ = now;
    RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,   // ms
            "偏移纠正角度 %f",
            gimbal_big_drift_
        );
}

void Node::handleSetSentryPosture(
    const std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Request> request,
    std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Response> response)
{
    // 合法姿态值: 1=进攻, 2=防御, 3=移动
    if (request->posture < 1 || request->posture > 3) {
        response->accepted = false;
        response->message = "Invalid posture value: " + std::to_string(request->posture) + " (must be 1, 2, or 3)";
        RCLCPP_WARN(get_logger(), "Rejected invalid posture: %u", request->posture);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        current_robot_posture_state_.data.posture = request->posture;
    }

    RCLCPP_INFO(get_logger(), "SentryPosture service called: posture=%u override=%d",
                request->posture, request->override_mode);

    // override 模式：不等待确认直接返回
    if (request->override_mode) {
        response->accepted = true;
        response->message = "Posture set (override, no confirm): " + std::to_string(request->posture);
        return;
    }

    // 阻塞等待下位机上报的姿态与目标一致
    posture_confirmed_.store(false, std::memory_order_release);
    pending_confirm_posture_.store(request->posture, std::memory_order_release);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(posture_confirm_timeout_ms_);

    while (!posture_confirmed_.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() >= deadline) {
            response->accepted = false;
            response->message = "Posture confirm timeout for " + std::to_string(request->posture);
            pending_confirm_posture_.store(0, std::memory_order_release);
            RCLCPP_WARN(get_logger(), "Posture confirm timeout: target=%u  ,sent posture= %u , current posture = %u", request->posture ,current_robot_posture_state_.data.posture , current_posture_);
            return;
        }
        std::this_thread::sleep_for(10ms);
    }

    pending_confirm_posture_.store(0, std::memory_order_release);
    response->accepted = true;
    response->message = "Posture confirmed: " + std::to_string(request->posture);
    RCLCPP_INFO(get_logger(), "Posture confirmed: %u", request->posture);
}

void Node::protectLoop()
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
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Open failed: %s", port_.c_str());
            }
        }
        std::this_thread::sleep_for(500ms);
    }
}

void Node::rxLoop()
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
        parseFrames(rxbuf);
    }
}

void Node::parseFrames(std::vector<uint8_t>& rxbuf)
{
    while (true) {
        if (rxbuf.size() < sizeof(rm_sentry_pp::HeaderFrame))
            return;

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

        const size_t body_len = 4 + static_cast<size_t>(hdr.data_len) + 1;
        const size_t frame_len = sizeof(rm_sentry_pp::HeaderFrame) + body_len;

        if (hdr.data_len == 0 || hdr.data_len > 64) {
            rxbuf.erase(rxbuf.begin());
            continue;
        }

        if (rxbuf.size() < frame_len)
            return;

        if (rxbuf[frame_len - 1] != rm_sentry_pp::HeaderFrame::EoF()) {
            rxbuf.erase(rxbuf.begin());
            continue;
        }

        if (hdr.id == rm_sentry_pp::ID_IMU) {
            if (hdr.data_len == sizeof(rm_sentry_pp::ReceiveImuData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveImuData)) {
                auto imu = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveImuData>(rxbuf.data());
                publishImu(imu);
            }
        }

        if(hdr.id == rm_sentry_pp::ID_ROBOT_INFO){
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveRobotInfoData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveRobotInfoData)){
                auto robot_info = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveRobotInfoData>(rxbuf.data());
                bool new_nav_status = robot_info.data.nav_status;
                // 检测 nav_status_ 上升沿：归中刚完成，记录 IMU 参考值
                if (!has_imu_centering_ref_ && new_nav_status && !nav_status_) {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    imu_at_centering_ = latest_imu_raw_yaw_;
                    {
                        std::lock_guard<std::mutex> lk2(odom_mtx_);
                        odom_at_centering_ = cached_odom_yaw_;
                    }
                    has_imu_centering_ref_ = true;
                    RCLCPP_INFO(get_logger(), "Gimbal_big centering detected (nav_status rising edge): imu_at_centering=%.4f rad",
                                imu_at_centering_);
                }
                nav_status_ = new_nav_status;
                publishRobotInfo(robot_info);
            }
        }
        if(hdr.id == rm_sentry_pp::ID_GAME_STATUS){
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveGameStatusData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveGameStatusData)){
                auto game_status = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveGameStatusData>(rxbuf.data());
                publishGameStatus(game_status);
            }
        }
        if(hdr.id == rm_sentry_pp::ID_ALL_ROBOT_HP){
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveAllRobotHpData::self_robot_hp) && frame_len == sizeof(rm_sentry_pp::ReceiveAllRobotHpData)){
                auto all_robot_hp = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveAllRobotHpData>(rxbuf.data());
                publishAllRobotHp(all_robot_hp);
            }
        }
        if(hdr.id == rm_sentry_pp::ID_ROBOT_LOCATION){
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveRobotLocation::data) && frame_len == sizeof(rm_sentry_pp::ReceiveRobotLocation)){
                auto robot_location = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveRobotLocation>(rxbuf.data());
                publishRobotLocation(robot_location);
            }
        }
        if(hdr.id == rm_sentry_pp::ID_RFID){
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveRfid::data) && frame_len == sizeof(rm_sentry_pp::ReceiveRfid)){
                auto rfid = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveRfid>(rxbuf.data());
                publishRfid(rfid);
            }
        }
        if(hdr.id == rm_sentry_pp::ID_ENEMY_LOCATION){
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveEnemyLocation::enemy) && frame_len == sizeof(rm_sentry_pp::ReceiveEnemyLocation)){
                auto enemy_location = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveEnemyLocation>(rxbuf.data());
                publishEnemyLocation(enemy_location);
            }
        }

        rxbuf.erase(rxbuf.begin(), rxbuf.begin() + frame_len);
    }
}

double Node::deg_to_rad_pi(double deg)
{
    double rad = deg * (M_PI / 180.0);
    return fmod(rad + M_PI, 2 * M_PI) - M_PI;
}

void Node::publishImu(const rm_sentry_pp::ReceiveImuData& imu_data)
{
    auto now = this->now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = imu_frame_;
    gimbal_big_yaw_ = deg_to_rad_pi(-imu_data.data.chassis_yaw); // 获取gimbal_big 的机械角 因为 chassis_yaw 是 gimbal_big 坐标系下的，所以 加个负号就可以了
    gimbal_yaw_ = deg_to_rad_pi(imu_data.data.gimbal_yaw); // 获取 gimbal_yaw 的机械角

    // 发布 gimbal_yaw TF: gimbal_big -> gimbal_yaw
    {
        geometry_msgs::msg::TransformStamped tf_gimbal_yaw;
        tf_gimbal_yaw.header.stamp = now;
        tf_gimbal_yaw.header.frame_id = "gimbal_big"; // "gimbal_big"
        tf_gimbal_yaw.child_frame_id = "gimbal_yaw";
        tf2::Quaternion q_gimbal_yaw;
        q_gimbal_yaw.setRPY(0, 0, gimbal_yaw_);
        tf_gimbal_yaw.transform.rotation = tf2::toMsg(q_gimbal_yaw);
        tf_broadcaster_->sendTransform(tf_gimbal_yaw);
    }

    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        latest_imu_raw_yaw_ = imu_data.data.yaw;

        // bool has_target = last_known_target_.valid.load(std::memory_order_relaxed) &&
        //                   last_known_target_.confidence.load(std::memory_order_relaxed) > min_confidence_threshold_;
        // 
        // gimbal_yaw IMU 漂移修正：当gimbal_yaw 云台回中且没有跟踪目标时，使用机械角校准 IMU   ,这是 gimbal_yaw 的修正，不是  gimbal_big 的
        // if (std::abs(gimbal_yaw_) < IMU_CALIBRATION_THRESHOLD &&
        //     !has_target &&
        //     !is_calibrating_imu_ &&
        //     (now - last_imu_calibration_time_).seconds() > IMU_CALIBRATION_INTERVAL) {

        //     double imu_yaw = target_gimbal_yaw_angle_;  // 这是 获取的 gimbal_yaw 的 yaw 值
        //     imu_yaw_offset_ = imu_yaw - gimbal_yaw_; // 这是 imu 的 yaw值 减去 gimbal_yaw 的 机械角 
        //     is_calibrating_imu_ = true;
        //     last_imu_calibration_time_ = now;

        //     RCLCPP_DEBUG(get_logger(), "IMU calibrated: offset=%.3f rad (gimbal_yaw=%.3f, imu_yaw=%.3f)",
        //                 imu_yaw_offset_, gimbal_yaw_, imu_yaw);
        // }

        // if (has_target) {
        //     is_calibrating_imu_ = false;
        // }

    }

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "gimbal_big_yaw=%.4f gimbal_yaw=%.4f rad, nav_status_=%d, has_centering_ref=%d",
        gimbal_big_yaw_, gimbal_yaw_, nav_status_, has_imu_centering_ref_);

    // double corrected_yaw = imu_data.data.yaw - imu_yaw_offset_;
    /*
    tf2::Quaternion q;
    q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
    imu.orientation = tf2::toMsg(q);

    imu.angular_velocity.x = imu_data.data.roll_vel;
    imu.angular_velocity.y = imu_data.data.pitch_vel;
    imu.angular_velocity.z = imu_data.data.yaw_vel;
    imu_pub_->publish(imu);*/
}

void Node::publishRobotInfo(const rm_sentry_pp::ReceiveRobotInfoData& robot_info_data)
{
    auto now = this->now();

    rm_decision_interfaces::msg::SentryPostureStatus posture_msg;
    //posture_msg.reported_posture = robot_info_data.data.posture;
    posture_msg.current_posture = robot_info_data.data.posture;

    posture_pub_->publish(posture_msg);

    // 姿态确认：下位机上报姿态与等待中的目标一致时，触发 service 返回
    current_posture_ = posture_msg.current_posture;
    uint8_t pending = pending_confirm_posture_.load(std::memory_order_acquire);
    if (pending != 0 && robot_info_data.data.posture == pending) {
        posture_confirmed_.store(true, std::memory_order_release);
    }

    rm_decision_interfaces::msg::RobotStatus robot_status_msg;
    robot_status_msg.robot_id = robot_info_data.data.id;
    robot_status_msg.team_color = robot_info_data.data.color;
    robot_status_msg.is_attacked = robot_info_data.data.attacked;
    robot_status_msg.current_hp = robot_info_data.data.hp;
    robot_status_msg.shot_allowance = robot_info_data.data.shot_allowance;
    robot_status_msg.shooter_heat = robot_info_data.data.heat_limit - robot_info_data.data.heat; // 这里转换成剩余可射击量，更直观一些

    robot_status_pub_->publish(robot_status_msg);
    std_msgs::msg::Bool start_save_map_msg;
    start_save_map_msg.data = robot_info_data.data.start_save_map;
    start_save_map_pub_->publish(start_save_map_msg);
}

void Node::publishGameStatus(const rm_sentry_pp::ReceiveGameStatusData& game_status_data)
{
    auto now = this->now();

    rm_decision_interfaces::msg::GameStatus game_status_msg;
    game_status_msg.game_progress = game_status_data.data.game_progress;
    game_status_msg.stage_remain_time = game_status_data.data.stage_remain_time;
    game_status_pub_->publish(game_status_msg);
}

void Node::publishAllRobotHp(const rm_sentry_pp::ReceiveAllRobotHpData& all_robot_hp_data)
{
    auto now = this->now();

    rm_decision_interfaces::msg::SelfRobotHP self_robot_hp_msg;
    self_robot_hp_msg.hero_hp = all_robot_hp_data.self_robot_hp.hero_hp;
    self_robot_hp_msg.engineer_hp = all_robot_hp_data.self_robot_hp.engineer_hp;
    self_robot_hp_msg.standard_3_hp = all_robot_hp_data.self_robot_hp.standard_3_hp;
    self_robot_hp_msg.standard_4_hp = all_robot_hp_data.self_robot_hp.standard_4_hp;
    self_robot_hp_msg.sentry_hp = all_robot_hp_data.self_robot_hp.sentry_hp;
    self_robot_hp_msg.outpost_hp = all_robot_hp_data.self_robot_hp.outpost_hp;
    self_robot_hp_msg.base_hp = all_robot_hp_data.self_robot_hp.base_hp;

    all_robot_hp_pub_->publish(self_robot_hp_msg);
}

void Node::publishRobotLocation(const rm_sentry_pp::ReceiveRobotLocation& robot_location_data)
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
    robot_location_msg.sentry_x = robot_location_data.data.sentry_x;
    robot_location_msg.sentry_y = robot_location_data.data.sentry_y;

    robot_location_pub_->publish(robot_location_msg);
}

void Node::publishEnemyLocation(const rm_sentry_pp::ReceiveEnemyLocation& enemy_location_data)
{
    auto now = this->now();
    rm_decision_interfaces::msg::EnemyLocation enemy_location_msg;
    enemy_location_msg.hero_x = enemy_location_data.enemy.hero_x;
    enemy_location_msg.hero_y = enemy_location_data.enemy.hero_y;
    enemy_location_msg.engineer_x = enemy_location_data.enemy.engineer_x;
    enemy_location_msg.engineer_y = enemy_location_data.enemy.engineer_y;
    enemy_location_msg.standard_3_x = enemy_location_data.enemy.standard_3_x;
    enemy_location_msg.standard_3_y = enemy_location_data.enemy.standard_3_y;
    enemy_location_msg.standard_4_x = enemy_location_data.enemy.standard_4_x;
    enemy_location_msg.standard_4_y = enemy_location_data.enemy.standard_4_y;
    enemy_location_msg.sentry_x = enemy_location_data.enemy.sentry_x;
    enemy_location_msg.sentry_y = enemy_location_data.enemy.sentry_y;

    enemy_location_pub_->publish(enemy_location_msg);
}

void Node::publishRfid(const rm_sentry_pp::ReceiveRfid& rfid_msg){
    auto now = this->now();

    #define GET_BIT(x,n) ((x) >> (n) & 0x1)
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

    rfid_pub_->publish(rfid_parse);
}

void Node::chiralLoop()
{
    rclcpp::Rate loop_rate(100);
    while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed)) {
        if (!chiral_reader_) {
            std::this_thread::sleep_for(100ms);
            continue;
        }

        if (auto data = chiral_reader_->read_new()) {
            publishTargetTracking(*data);
        } else {
            std::this_thread::sleep_for(100ms);
        }
        loop_rate.sleep();
    }
}

void Node::updateEnemyForbiddenArea(const rm_decision_interfaces::msg::EnemyForbiddenArea& msg)
{
    const auto idx = static_cast<size_t>(msg.armors_num);

    {
        std::lock_guard<std::mutex> lk(invincible_mtx_);

        if (idx >= std::size(invincible_cache_)) {
            RCLCPP_WARN(get_logger(),
                "EnemyForbiddenArea armor index out of range: %zu", idx);
            return;
        }

        invincible_cache_[idx] = msg.is_forbidden;
    }

    if (chiral_reader_) {
        auto chiral_write = talos::chiral::navigation::NavigationData::create();
        {
            std::lock_guard<std::mutex> lk(invincible_mtx_);
            std::copy(std::begin(invincible_cache_), std::end(invincible_cache_),
                      std::begin(chiral_write.invincible));
        }
        chiral_reader_->write(chiral_write);
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Enemy forbidden state: armor=%zu forbidden=%d", idx, msg.is_forbidden);
}

bool Node::isArmorForbidden(talos::chiral::navigation::ArmorName armor_name)
{
    const auto idx = static_cast<size_t>(armor_name);
    std::lock_guard<std::mutex> lk(invincible_mtx_);

    if (idx >= std::size(invincible_cache_)) {
        return false;
    }

    return invincible_cache_[idx];
}
void Node::onRobotAreaStatus(const rm_decision_interfaces::msg::RobotAreaStatus& msg)
{
    if(msg.is_in_area && msg.area_name == robot_area_name_ && msg.matched_area_names == std::vector<std::string>{robot_area_name_})
    {
        // Handle the case when the robot is in the specified area
        perception_status_ = false;  // 进入起伏路段，认为无法感知敌人
        track_status_ = true;        // 进入起伏路段，开启底盘履带
        follow_gimbal_big_ = true;     // 进入起伏路段，跟随 gimbal_big
    }
    else
    {
    // Handle the case when the robot is outside the specified area
        perception_status_ = true;  // 离开起伏路段，认为可以感知敌人
        track_status_ = false;       // 离开起伏路段，关闭底盘履带
        follow_gimbal_big_ = false;    // 离开起伏路段，不跟随 gimbal_big
    }
}

void Node::publishTargetTracking(const talos::chiral::navigation::TalosData& talos_data)
{
    auto now = this->now();

    armor_interfaces::msg::Target target_msg;
    target_msg.header.stamp = now;
    target_msg.header.frame_id = "gimbal_yaw";

    // ===============================
    // 1. 先处理追踪状态
    // ===============================
    switch (talos_data.state.status) {
        case talos::chiral::navigation::TrackerStatus::Idle:
            target_msg.tracking = false;
            target_msg.tracking_status = 0;
            target_msg.confidence = 0.0;

            break;

        case talos::chiral::navigation::TrackerStatus::Detecting:
            target_msg.tracking = false;
            target_msg.tracking_status = 1;
            target_msg.confidence = 0.0;
            break;

        case talos::chiral::navigation::TrackerStatus::Tracking:
            target_msg.tracking = true;
            target_msg.tracking_status = 2;
            target_msg.confidence = 1.0;
            break;

        case talos::chiral::navigation::TrackerStatus::TempLost:
            target_msg.tracking = true;
            target_msg.tracking_status = 3;
            target_msg.confidence = 1.0;

            break;
    }

    // ===============================
    // 2. 如果当前正在追踪目标，使用视觉实时数据
    // ===============================
    if (talos_data.state.status == talos::chiral::navigation::TrackerStatus::Tracking || talos_data.state.status == talos::chiral::navigation::TrackerStatus::TempLost) {
        // ===============================
        // 1.5 禁区拦截：目标在禁区内则抑制追踪
        // ===============================
        if (talos_data.state_kind == talos::chiral::navigation::TargetStateKind::Robot
            && talos_data.state.name == talos::chiral::navigation::ArmorName::Two) {
            if (isArmorForbidden(talos_data.state.name)) {
                target_msg.tracking = false;
                target_msg.tracking_status = 0;
                target_msg.confidence = 0.0;
                target_msg.position.x = 0.0;
                target_msg.position.y = 0.0;
                target_msg.position.z = 0.0;
                target_msg.velocity.x = 0.0;
                target_msg.velocity.y = 0.0;
                target_msg.velocity.z = 0.0;
                target_msg.id = "forbidden";
                target_msg.armors_num = static_cast<int>(talos_data.state.robot.armor_num);

                visualization_msgs::msg::Marker del_marker;
                del_marker.header.stamp = now;
                del_marker.header.frame_id = "gimbal_yaw";
                del_marker.ns = "enemy";
                del_marker.id = 0;
                del_marker.action = visualization_msgs::msg::Marker::DELETE;
                enemy_marker_pub_->publish(del_marker);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Target is in forbidden area, suppress tracking");

                target_tracking_pub_->publish(target_msg);
                return;
            }
        }

        double enemy_x = 0.0;
        double enemy_y = 0.0;
        double enemy_z = 0.0;

        double enemy_vx = 0.0;
        double enemy_vy = 0.0;
        double enemy_vz = 0.0;

        // -------------------------------
        // 2.1 读取视觉给出的敌人位置
        //
        // 这里你已经确认：
        // 视觉给的是 ROS 坐标系下，敌人相对于 gimbal_yaw 的位置
        //
        // 即：
        // x：云台当前朝向前方
        // y：云台左侧
        // z：上方
        // -------------------------------
        if (talos_data.state_kind == talos::chiral::navigation::TargetStateKind::Robot) {
            enemy_x = talos_data.state.robot.position.x;
            enemy_y = talos_data.state.robot.position.y;
            enemy_z = talos_data.state.robot.position.z;

            enemy_vx = talos_data.state.robot.velocity.x;
            enemy_vy = talos_data.state.robot.velocity.y;
            enemy_vz = talos_data.state.robot.velocity.z;
        } 
        // 关闭前哨站信息，改为在巡逻点处停留45秒，增加哨兵的攻击机会，避免哨兵乱跑
        // else if (talos_data.state_kind == talos::chiral::navigation::TargetStateKind::Outpost) {
        //     enemy_x = talos_data.state.outpost.position.x;
        //     enemy_y = talos_data.state.outpost.position.y;
        //     enemy_z = talos_data.state.outpost.position.z;

        //     enemy_vx = 0.0;
        //     enemy_vy = 0.0;
        //     enemy_vz = 0.0;
        //     // 对于前哨站，暂时没有速度信息，先默认是静止的
        //     // 对于前哨站的补充是要哨兵进行攻击判定的，所以添加了一个特殊标记 armors_num = 6 来区分前哨站和机器人目标，在攻击判定时如果是前哨站就不考虑速度因素，直接进行攻击判定
        // } 
        else {
            target_msg.tracking = false;
            target_msg.confidence = 0.0;
            target_tracking_pub_->publish(target_msg);
            return;
        }

        // 直接使用 gimbal_yaw 坐标系下的位置和速度（视觉数据已在 gimbal_yaw 系下）
        target_msg.position.x = enemy_x;
        target_msg.position.y = enemy_y;
        target_msg.position.z = enemy_z;

        target_msg.velocity.x = enemy_vx;
        target_msg.velocity.y = enemy_vy;
        target_msg.velocity.z = enemy_vz;
    }

    // ===============================
    // 9. 没有有效目标
    // ===============================
    else {
        target_msg.position.x = 0.0;
        target_msg.position.y = 0.0;
        target_msg.position.z = 0.0;

        target_msg.velocity.x = 0.0;
        target_msg.velocity.y = 0.0;
        target_msg.velocity.z = 0.0;
        target_msg.confidence = 0.0;  // 没有目标时置信度显式为0，去除前哨站
        target_tracking_pub_->publish(target_msg);
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.stamp = now;
        del_marker.header.frame_id = "gimbal_yaw";
        del_marker.ns = "enemy";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETE;
        enemy_marker_pub_->publish(del_marker);
        return;
    }

    // ===============================
    // 10. 补充机器人目标的装甲板信息
    // ===============================
    if (talos_data.state_kind == talos::chiral::navigation::TargetStateKind::Robot) {
        target_msg.yaw = talos_data.state.robot.yaw;
        target_msg.v_yaw = talos_data.state.robot.v_yaw;

        target_msg.radius_1 = talos_data.state.robot.radius0;
        target_msg.radius_2 = talos_data.state.robot.radius1;
        target_msg.dz = talos_data.state.robot.z1;
        target_msg.armors_num = talos_data.state.robot.armor_num;

        std::ostringstream oss;
        switch (talos_data.state.name) {
            case talos::chiral::navigation::ArmorName::Sentry:
                oss << "sentry";
                break;
            case talos::chiral::navigation::ArmorName::One:
                oss << "hero";
                break;
            case talos::chiral::navigation::ArmorName::Two:
                oss << "engineer";
                break;
            case talos::chiral::navigation::ArmorName::Three:
                oss << "standard_3";
                break;
            case talos::chiral::navigation::ArmorName::Four:
                oss << "standard_4";
                break;
            case talos::chiral::navigation::ArmorName::Five:
                oss << "standard_5";
                break;
            case talos::chiral::navigation::ArmorName::Base:
                oss << "base";
                break;
            default:
                oss << "unknown";
                break;
        }

        target_msg.id = oss.str();
    }

    // ===============================
    // 11. 补充前哨站目标信息
    // ===============================
    else if (talos_data.state_kind == talos::chiral::navigation::TargetStateKind::Outpost) {
        target_msg.yaw = talos_data.state.outpost.yaw;
        target_msg.v_yaw = talos_data.state.outpost.v_yaw;

        target_msg.id = "outpost";
        target_msg.armors_num = 6; // 前哨站特殊标记为 6
    }

    // ===============================
    // 12. 发布敌人位置 marker (黄色球, 0.5m半径)
    // ===============================
    if (target_msg.tracking || target_msg.tracking_status == 2 || target_msg.tracking_status == 3) {
        visualization_msgs::msg::Marker enemy_marker;
        enemy_marker.header.stamp = now;
        enemy_marker.header.frame_id = "gimbal_yaw";
        enemy_marker.ns = "enemy";
        enemy_marker.id = 0;
        enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
        enemy_marker.action = visualization_msgs::msg::Marker::ADD;
        enemy_marker.pose.position.x = target_msg.position.x;
        enemy_marker.pose.position.y = target_msg.position.y;
        enemy_marker.pose.position.z = target_msg.position.z;
        enemy_marker.pose.orientation.w = 1.0;
        enemy_marker.scale.x = 1.0;
        enemy_marker.scale.y = 1.0;
        enemy_marker.scale.z = 1.0;
        enemy_marker.color.r = 1.0f;
        enemy_marker.color.g = 1.0f;
        enemy_marker.color.b = 0.0f;
        enemy_marker.color.a = 0.8f;
        enemy_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        enemy_marker_pub_->publish(enemy_marker);
    } else {
        visualization_msgs::msg::Marker enemy_marker;
        enemy_marker.header.stamp = now;
        enemy_marker.header.frame_id = "gimbal_yaw";
        enemy_marker.ns = "enemy";
        enemy_marker.id = 0;
        enemy_marker.action = visualization_msgs::msg::Marker::DELETE;
        enemy_marker_pub_->publish(enemy_marker);
    }

    // ===============================
    // 13. 发布 gimbal_yaw 坐标系下的目标
    // ===============================
    target_tracking_pub_->publish(target_msg);
}
void Node::txLoop()
{
    rclcpp::WallRate loop_rate { std::chrono::milliseconds(send_period_ms_) };
    while (rclcpp::ok() && !exit_.load()) {
        if (!is_port_ok_.load()) {
            loop_rate.sleep();
            continue;
        }

        {
            rm_sentry_pp::SendRobotCmdData pkt {};
            {
                std::lock_guard<std::mutex> lk(tx_mtx_);
                rm_sentry_pp::fillHeader(pkt, rm_sentry_pp::ID_ROBOT_CMD);
                pkt.frame_header.id = rm_sentry_pp::ID_ROBOT_CMD;
                pkt.time_stamp = nowMs();
                pkt.data.speed_vector = current_cmd_state_.data.speed_vector;

                double angle_age_ms = (this->now() - last_gimbal_angle_update_).seconds() * 1000.0;
                bool angle_valid = (angle_age_ms < gimbal_angle_timeout_ms_);

                if (angle_valid) {
                    double dt_since = (this->now() - last_drift_update_).seconds();
                    double predicted_drift = gimbal_big_drift_ + gimbal_big_drift_rate_ * dt_since;
                    gimbal_big_yaw_angle_state_ = gimbal_big_yaw_angle_ + predicted_drift;
                    pkt.data.gimbal_big.yaw_angle = gimbal_big_yaw_angle_state_;
                        RCLCPP_DEBUG_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,   // ms
                        "发布角度%f",
                        pkt.data.gimbal_big.yaw_angle
                    );
                    pkt.data.gimbal_big.yaw_vel = 0.0f;
                } else {
                    pkt.data.gimbal_big.yaw_angle = gimbal_big_yaw_angle_state_;  // 角度过期，继续发布预测的角度，但不再更新预测值（相当于冻结在最后一个有效角度）。去除了速度控制
                    pkt.data.gimbal_big.yaw_vel = 0.0f;
                }

                pkt.eof = rm_sentry_pp::HeaderFrame::EoF();
            }

            auto bytes = rm_sentry_pp::toVector(pkt);
            std::lock_guard<std::mutex> lk(port_mtx_);
            if (!sp_.writeAll(bytes.data(), bytes.size())) {
                is_port_ok_.store(false);
            }
        }

        {
            rm_sentry_pp::SendRobotPostureData pkt {};
            {
                std::lock_guard<std::mutex> lk(tx_mtx_);
                rm_sentry_pp::fillHeader(pkt, rm_sentry_pp::ID_ROBOT_POSTURE);
                pkt.frame_header.id = rm_sentry_pp::ID_ROBOT_POSTURE;
                pkt.time_stamp = nowMs();
                pkt.data.posture = current_robot_posture_state_.data.posture;
                pkt.data.follow_gimbal_big = follow_gimbal_big_;
                pkt.data.track_status = track_status_;
                pkt.data.perception_status = perception_status_;
                pkt.data.start_gimbal_big_spin = start_gimbal_big_spin_;
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

double Node::calculateDecayedConfidence(double current_confidence, double dt) {
    return std::max(0.0, current_confidence * std::exp(-confidence_decay_lambda_ * dt));
}

void Node::onOdom(const nav_msgs::msg::Odometry& msg)
{
    std::lock_guard<std::mutex> lk(odom_mtx_);
    cached_odom_x_ = msg.pose.pose.position.x;
    cached_odom_y_ = msg.pose.pose.position.y;

    tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, cached_odom_yaw_);

    cached_odom_vx_ = msg.twist.twist.linear.x;
    cached_odom_vy_ = msg.twist.twist.linear.y;
    last_odom_time_ = this->now();

    // odom 稳定检测
    if (!odom_stable_) {
        odom_stable_count_++;
        if (odom_stable_count_ >= ODOM_STABLE_REQUIRED) {
            odom_stable_ = true;
            RCLCPP_INFO(get_logger(), "Odometry stabilized after %d frames", odom_stable_count_);
        }
    }
}

bool Node::getChassisPoseInMap(double& x, double& y, double& yaw)
{
    std::lock_guard<std::mutex> lk(odom_mtx_);
    if ((this->now() - last_odom_time_).seconds() * 1000.0 > odom_timeout_ms_) return false;

    x = cached_odom_x_;
    y = cached_odom_y_;
    yaw = cached_odom_yaw_;

    if (relocalization_mode_) {
        std::lock_guard<std::mutex> lk2(map_odom_mtx_);
        if (!has_cached_map_to_odom_) return false;

        double cy = std::cos(cached_map_to_odom_yaw_);
        double sy = std::sin(cached_map_to_odom_yaw_);
        double rx = x * cy - y * sy;
        double ry = x * sy + y * cy;
        x = rx + cached_map_to_odom_x_;
        y = ry + cached_map_to_odom_y_;
        yaw = cached_map_to_odom_yaw_;
    }
    return true;
}

void Node::updateMapToOdom()
{
    if (!tf_buffer_) return;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
        std::lock_guard<std::mutex> lk(map_odom_mtx_);
        cached_map_to_odom_x_ = tf.transform.translation.x;
        cached_map_to_odom_y_ = tf.transform.translation.y;
        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        double r, p;
        tf2::Matrix3x3(q).getRPY(r, p, cached_map_to_odom_yaw_);
        has_cached_map_to_odom_ = true;
    } catch (const tf2::TransformException&) {}
}

} // namespace rm_sentry_pp_nocrc_serial

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rm_sentry_pp_nocrc_serial::Node>(rclcpp::NodeOptions {}));
    rclcpp::shutdown();
    return 0;
}
