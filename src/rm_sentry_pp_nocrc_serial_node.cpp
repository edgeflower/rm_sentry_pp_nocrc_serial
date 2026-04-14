#include "rm_sentry_pp_nocrc_serial/node.hpp"
#include <rclcpp/logging.hpp>

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
    gimbal_follow_path_topic_ = declare_parameter<std::string>("gimbal_follow_path_topic", "plan");
    gimbal_follow_lookahead_ = declare_parameter<double>("gimbal_follow_lookahead", 1.5);
    gimbal_lookahead_base_ = declare_parameter<double>("gimbal_lookahead_base", 0.8);
    gimbal_lookahead_k_ = declare_parameter<double>("gimbal_lookahead_k", 0.4);
    gimbal_yaw_smooth_alpha_ = declare_parameter<double>("gimbal_yaw_smooth_alpha", 0.3);

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

    // 机器人姿态状态
    posture_pub_ = create_publisher<rm_decision_interfaces::msg::SentryPostureStatus>("sentry_posture_status", 10);

    // 机器人状态消息
    robot_status_pub_ = create_publisher<rm_decision_interfaces::msg::RobotStatus>("robot_status", 10);

    // 比赛状态信息
    game_status_pub_ = create_publisher<rm_decision_interfaces::msg::GameStatus>("game_status", 10);

    // 所有机器人血量消息
    all_robot_hp_pub_ = create_publisher<rm_decision_interfaces::msg::AllRobotHP>("all_robot_hp", 10);

    // 机器人位置信息消息
    robot_location_pub_ = create_publisher<rm_decision_interfaces::msg::FriendLocation>("robot_location", 10);

    // rfid 信息
    rfid_pub_ = create_publisher<rm_decision_interfaces::msg::RFIDParse>("rfid",10);

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
    last_gimbal_angle_update_ = node_start_;
    last_path_time_ = node_start_;
    last_drift_update_ = node_start_;
    last_odom_time_ = node_start_;

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
    follow_gimbal_big_ = msg.follow_gimbal_big;
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
    std::lock_guard<std::mutex> lk(tx_mtx_);
    current_robot_posture_state_.data.posture = request->posture;
    tx_posture_pending_ = true;

    response->accepted = true;
    response->message = "Posture set to " + std::to_string(request->posture);

    RCLCPP_INFO(get_logger(), "SentryPosture service called: posture=%u override=%d",
                request->posture, request->override_mode);
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
            if(hdr.data_len == sizeof(rm_sentry_pp::ReceiveAllRobotHpData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveAllRobotHpData)){
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


    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        latest_imu_raw_yaw_ = imu_data.data.yaw;

        bool has_target = last_known_target_.valid.load(std::memory_order_relaxed) &&
                          last_known_target_.confidence.load(std::memory_order_relaxed) > min_confidence_threshold_;
        
        // gimbal_yaw IMU 漂移修正：当gimbal_yaw 云台回中且没有跟踪目标时，使用机械角校准 IMU   ,这是 gimbal_yaw 的修正，不是  gimbal_big 的
        if (std::abs(gimbal_yaw_) < IMU_CALIBRATION_THRESHOLD &&
            !has_target &&
            !is_calibrating_imu_ &&
            (now - last_imu_calibration_time_).seconds() > IMU_CALIBRATION_INTERVAL) {

            double imu_yaw = target_gimbal_yaw_angle_;  // 这是 获取的 gimbal_yaw 的 yaw 值
            imu_yaw_offset_ = imu_yaw - gimbal_yaw_; // 这是 imu 的 yaw值 减去 gimbal_yaw 的 机械角 
            is_calibrating_imu_ = true;
            last_imu_calibration_time_ = now;

            RCLCPP_DEBUG(get_logger(), "IMU calibrated: offset=%.3f rad (gimbal_yaw=%.3f, imu_yaw=%.3f)",
                        imu_yaw_offset_, gimbal_yaw_, imu_yaw);
        }

        if (has_target) {
            is_calibrating_imu_ = false;
        }

        // gimbal_big 漂移校正：检测归中时刻，记录 IMU 参考值
        if (!has_imu_centering_ref_ && std::abs(gimbal_big_yaw_) < IMU_CALIBRATION_THRESHOLD ) {
            imu_at_centering_ = imu_data.data.yaw;
            double odom_yaw_copy;
            {
                std::lock_guard<std::mutex> lk(odom_mtx_);
                odom_yaw_copy = cached_odom_yaw_;
            }
            odom_at_centering_ = odom_yaw_copy;
            has_imu_centering_ref_ = true;
            RCLCPP_INFO(get_logger(), "Gimbal_big centering detected: imu_at_centering=%.4f rad", imu_at_centering_);
        }
    }

    // double corrected_yaw = imu_data.data.yaw - imu_yaw_offset_;

    tf2::Quaternion q;
    q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
    imu.orientation = tf2::toMsg(q);

    imu.angular_velocity.x = imu_data.data.roll_vel;
    imu.angular_velocity.y = imu_data.data.pitch_vel;
    imu.angular_velocity.z = imu_data.data.yaw_vel;
    imu_pub_->publish(imu);
}

void Node::publishRobotInfo(const rm_sentry_pp::ReceiveRobotInfoData& robot_info_data)
{
    auto now = this->now();

    rm_decision_interfaces::msg::SentryPostureStatus posture_msg;
    //posture_msg.reported_posture = robot_info_data.data.posture;
    posture_msg.current_posture = robot_info_data.data.posture;

    posture_pub_->publish(posture_msg);

    rm_decision_interfaces::msg::RobotStatus robot_status_msg;
    robot_status_msg.robot_id = robot_info_data.data.id;
    robot_status_msg.team_color = robot_info_data.data.color;
    robot_status_msg.is_attacked = robot_info_data.data.attacked;
    robot_status_msg.current_hp = robot_info_data.data.hp;
    robot_status_msg.shot_allowance = robot_info_data.data.shot_allowance;
    robot_status_msg.shooter_heat = robot_info_data.data.heat_limit;

    robot_status_pub_->publish(robot_status_msg);
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

    all_robot_hp_pub_->publish(all_robot_hp_msg);
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
    robot_location_msg.standard_5_x = robot_location_data.data.sentry_x;
    robot_location_msg.standard_5_y = robot_location_data.data.sentry_y;

    robot_location_pub_->publish(robot_location_msg);
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

void Node::publishTargetTracking(const talos::chrial::TalosData& talos_data)
{
    auto now = this->now();

    armor_interfaces::msg::Target target_msg;
    target_msg.header.stamp = now;
    target_msg.header.frame_id = "map";

    switch (talos_data.state.status) {
        case talos::chrial::TrackerStatus::Idle:
            target_msg.tracking = false;
            target_msg.tracking_status = 0;
            last_known_target_.valid.store(false, std::memory_order_relaxed);
            last_known_target_.confidence.store(0.0, std::memory_order_relaxed);
            break;
        case talos::chrial::TrackerStatus::Detecting:
            target_msg.tracking = false;
            target_msg.tracking_status = 1;
            if (last_known_target_.valid) {
                double time_since_last = (now - last_known_target_.last_update_time).seconds();
                last_known_target_.confidence.store(
                    calculateDecayedConfidence(last_known_target_.confidence.load(std::memory_order_relaxed), time_since_last),
                    std::memory_order_relaxed);
            }
            target_msg.confidence = last_known_target_.confidence.load(std::memory_order_relaxed);
            break;
        case talos::chrial::TrackerStatus::Tracking:
            target_msg.tracking = true;
            target_msg.tracking_status = 2;
            target_msg.confidence = 1.0;
            break;
        case talos::chrial::TrackerStatus::TempLost:
            target_msg.tracking = false;
            target_msg.tracking_status = 3;
            if (last_known_target_.valid.load(std::memory_order_relaxed)) {
                double time_since_last = (now - last_known_target_.last_update_time).seconds();
                last_known_target_.confidence.store(
                    calculateDecayedConfidence(last_known_target_.confidence.load(std::memory_order_relaxed), time_since_last),
                    std::memory_order_relaxed);
                target_msg.confidence = last_known_target_.confidence.load(std::memory_order_relaxed);
            } else {
                target_msg.confidence = 0.0;
            }
            break;
    }

    {
        tf2::Quaternion q_yaw_to_big(
            talos_data.gimbal_link.rotation.x,
            talos_data.gimbal_link.rotation.y,
            talos_data.gimbal_link.rotation.z,
            talos_data.gimbal_link.rotation.w
        );
        double r_unused, p_unused, yaw_from_tf;
        tf2::Matrix3x3(q_yaw_to_big).getRPY(r_unused, p_unused, yaw_from_tf);
        while (yaw_from_tf > M_PI) yaw_from_tf -= 2 * M_PI;
        while (yaw_from_tf < -M_PI) yaw_from_tf += 2 * M_PI;
        std::lock_guard<std::mutex> lk(tx_mtx_);
        target_gimbal_yaw_angle_ = static_cast<float>(yaw_from_tf);
        imu_data_cached = static_cast<float>(yaw_from_tf);
        last_gimbal_angle_update_ = now;
    }

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
            target_tracking_pub_->publish(target_msg);
            return;
        }

        tf2::Vector3 enemy_gimbal_yaw(enemy_x, enemy_y, enemy_z);
        tf2::Vector3 velocity_gimbal_yaw(enemy_vx, enemy_vy, enemy_vz);

        tf2::Quaternion q_yaw_to_big(
            talos_data.gimbal_link.rotation.x,
            talos_data.gimbal_link.rotation.y,
            talos_data.gimbal_link.rotation.z,
            talos_data.gimbal_link.rotation.w
        );
        tf2::Vector3 enemy_gimbal_big = tf2::quatRotate(q_yaw_to_big, enemy_gimbal_yaw);
        tf2::Vector3 velocity_gimbal_big = tf2::quatRotate(q_yaw_to_big, velocity_gimbal_yaw);

        {
            double chassis_x, chassis_y, chassis_yaw;
            if (!getChassisPoseInMap(chassis_x, chassis_y, chassis_yaw)) {
                target_tracking_pub_->publish(target_msg);
                return;
            }

            double gimbal_big_yaw = chassis_yaw;  // chassis ≡ gimbal_big，odom yaw 即真实朝向
            while (gimbal_big_yaw > M_PI) gimbal_big_yaw -= 2 * M_PI;
            while (gimbal_big_yaw < -M_PI) gimbal_big_yaw += 2 * M_PI;

            tf2::Quaternion q_map_to_big;
            q_map_to_big.setRPY(0, 0, gimbal_big_yaw);
            tf2::Vector3 t_map_to_big(chassis_x, chassis_y, 0);

            tf2::Vector3 enemy_map_rotated = tf2::quatRotate(q_map_to_big, enemy_gimbal_big);
            tf2::Vector3 enemy_map = enemy_map_rotated + t_map_to_big;
            tf2::Vector3 velocity_map = tf2::quatRotate(q_map_to_big, velocity_gimbal_big);

            target_msg.position.x = enemy_map.x();
            target_msg.position.y = enemy_map.y();
            target_msg.position.z = enemy_map.z();
            target_msg.velocity.x = velocity_map.x();
            target_msg.velocity.y = velocity_map.y();
            target_msg.velocity.z = velocity_map.z();

            const double CHASSIS_RESPONSE_DELAY = 0.2;
            double predicted_x = enemy_map.x() + velocity_map.x() * CHASSIS_RESPONSE_DELAY;
            double predicted_y = enemy_map.y() + velocity_map.y() * CHASSIS_RESPONSE_DELAY;
            double predicted_z = 0.0;

            last_known_target_.valid.store(true, std::memory_order_relaxed);
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
            last_known_target_.confidence.store(1.0, std::memory_order_relaxed);

            RCLCPP_DEBUG(get_logger(), "Target predicted: pos=(%.2f, %.2f) -> pred=(%.2f, %.2f)",
                        enemy_map.x(), enemy_map.y(), predicted_x, predicted_y);
        }
    } else if (talos_data.state.status == talos::chrial::TrackerStatus::TempLost &&
               last_known_target_.valid.load(std::memory_order_relaxed) &&
               last_known_target_.confidence.load(std::memory_order_relaxed) > min_confidence_threshold_) {
        target_msg.position.x = last_known_target_.predicted_x;
        target_msg.position.y = last_known_target_.predicted_y;
        target_msg.position.z = last_known_target_.predicted_z;
        target_msg.velocity.x = last_known_target_.velocity_x;
        target_msg.velocity.y = last_known_target_.velocity_y;
        target_msg.velocity.z = last_known_target_.velocity_z;

        double time_since_last = (now - last_known_target_.last_update_time).seconds();
        target_msg.position.x += last_known_target_.velocity_x * time_since_last;
        target_msg.position.y += last_known_target_.velocity_y * time_since_last;
        target_msg.position.z = 0.0;

        target_msg.yaw = last_known_target_.yaw;
        target_msg.v_yaw = last_known_target_.v_yaw;
        target_msg.radius_1 = last_known_target_.radius_1;
        target_msg.radius_2 = last_known_target_.radius_2;
        target_msg.dz = last_known_target_.dz;
        target_msg.armors_num = last_known_target_.armors_num;
        target_msg.id = last_known_target_.id;

        RCLCPP_DEBUG(get_logger(), "Target TempLost: using predicted position (%.2f, %.2f) with confidence=%.2f",
                    target_msg.position.x, target_msg.position.y,
                    last_known_target_.confidence.load(std::memory_order_relaxed));
    } else {
        target_msg.position.x = 0;
        target_msg.position.y = 0;
        target_msg.position.z = 0;
        target_msg.velocity.x = 0;
        target_msg.velocity.y = 0;
        target_msg.velocity.z = 0;
        target_msg.confidence = 0.0;
    }

    if (talos_data.state_kind == talos::chrial::TargetStateKind::Robot) {
        target_msg.yaw = talos_data.state.robot.yaw;
        target_msg.v_yaw = talos_data.state.robot.v_yaw;

        target_msg.radius_1 = talos_data.state.robot.radius0;
        target_msg.radius_2 = talos_data.state.robot.radius1;
        target_msg.dz = talos_data.state.robot.z1;

        target_msg.armors_num = talos_data.state.robot.armor_num;

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
        target_msg.yaw = talos_data.state.outpost.yaw;
        target_msg.v_yaw = talos_data.state.outpost.v_yaw;

        target_msg.id = "outpost";
        target_msg.armors_num = 3;

        if (talos_data.state.status == talos::chrial::TrackerStatus::Tracking) {
            last_known_target_.yaw = target_msg.yaw;
            last_known_target_.v_yaw = target_msg.v_yaw;
            last_known_target_.radius_1 = 0;
            last_known_target_.radius_2 = 0;
            last_known_target_.dz = 0;
            last_known_target_.armors_num = target_msg.armors_num;
            last_known_target_.id = target_msg.id;
            last_known_target_.target_kind = talos::chrial::TargetStateKind::Outpost;
        }
    }

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
                    pkt.data.gimbal_big.yaw_angle = gimbal_big_yaw_angle_ + predicted_drift;
                    pkt.data.gimbal_big.yaw_vel = 0.0f;
                } else {
                    pkt.data.gimbal_big.yaw_angle = 0.0f;
                    pkt.data.gimbal_big.yaw_vel = current_cmd_state_.data.gimbal_big.yaw_vel;
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
