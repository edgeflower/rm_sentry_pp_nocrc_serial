#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl/context.h>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_decision_interfaces/msg/detail/enemy_forbidden_area__struct.hpp>
#include <rm_decision_interfaces/msg/detail/enemy_location__struct.hpp>
#include <rm_decision_interfaces/msg/detail/robot_area_status__struct.hpp>
#include <rm_decision_interfaces/msg/detail/self_robot_hp__struct.hpp>
#include <rm_decision_interfaces/msg/detail/sentry_posture_status__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sys/types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <armor_interfaces/msg/target.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <cmath>

#include "chiral/chiral_endpoint.hpp"
#include "chiral/navigation.hpp"
#include "rm_sentry_pp_nocrc_serial/packet.hpp"
#include "rm_sentry_pp_nocrc_serial/serial_port.hpp"
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
#include<rm_decision_interfaces/msg/enemy_location.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rm_decision_interfaces/msg/enemy_forbidden_area.hpp>
#include <rm_decision_interfaces/msg/robot_area_status.hpp>
#include <rm_decision_interfaces/msg/self_robot_hp.hpp>

namespace rm_sentry_pp_nocrc_serial {

class Node : public rclcpp::Node {
public:
    explicit Node(const rclcpp::NodeOptions& options);
    ~Node() override;

private:
    uint32_t nowMs() const;

    void onCmd(const geometry_msgs::msg::Twist& msg);  // 接收控制话题
    void onRobotControl(const rm_decision_interfaces::msg::RobotControl& msg);  // 接收控制话题
    void onPath(const nav_msgs::msg::Path& msg); // 获取 global_plan

    void updateGimbalFromCachedPath(); // 维护gimbal_big 方向
    void updateDriftCorrection(); // 维护
    void onOdom(const nav_msgs::msg::Odometry& msg);
    bool getChassisPoseInMap(double& x, double& y, double& yaw);
    void updateMapToOdom();

    void handleSetSentryPosture(
        const std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Request> request,
        std::shared_ptr<rm_decision_interfaces::srv::SetSentryPosture::Response> response);

    void protectLoop();
    void rxLoop();
    void parseFrames(std::vector<uint8_t>& rxbuf);
    void chiralLoop();
    void txLoop();

    double deg_to_rad_pi(double deg);

    void publishImu(const rm_sentry_pp::ReceiveImuData& imu_data);
    void publishRobotInfo(const rm_sentry_pp::ReceiveRobotInfoData& robot_info_data);
    void publishGameStatus(const rm_sentry_pp::ReceiveGameStatusData& game_status_data);
    void publishAllRobotHp(const rm_sentry_pp::ReceiveAllRobotHpData& all_robot_hp_data);
    void publishRobotLocation(const rm_sentry_pp::ReceiveRobotLocation& robot_location_data);
    void publishRfid(const rm_sentry_pp::ReceiveRfid& rfid_msg);
    void publishEnemyLocation(const rm_sentry_pp::ReceiveEnemyLocation& enemy_location_data);

    void publishTargetTracking(const talos::chiral::navigation::TalosData& talos_data);

    double calculateDecayedConfidence(double current_confidence, double dt);
    void updateEnemyForbiddenArea(const rm_decision_interfaces::msg::EnemyForbiddenArea& enemy_forbidden_area);
    void onRobotAreaStatus(const rm_decision_interfaces::msg::RobotAreaStatus& robot_area_status);
    bool isArmorForbidden(talos::chiral::navigation::ArmorName armor_name);

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
    int gimbal_angle_timeout_ms_ { 300 };
    std::string gimbal_follow_path_topic_;
    double gimbal_follow_lookahead_ { 1.5 };
    double gimbal_lookahead_base_ { 0.8 };
    double gimbal_lookahead_k_ { 0.4 };
    double gimbal_yaw_smooth_alpha_ { 0.3 };
    bool nav_status_ = false;
    std::string robot_area_name_;
    bool start_gimbal_big_spin_ { false };

    // Odometry parameters
    std::string odom_topic_;
    bool relocalization_mode_ { false };
    int odom_timeout_ms_ { 500 };

    // ros
    rclcpp::Time node_start_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<armor_interfaces::msg::Target>::SharedPtr target_tracking_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gimbal_yaw_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_point_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr enemy_marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotControl>::SharedPtr robot_control_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<rm_decision_interfaces::srv::SetSentryPosture>::SharedPtr set_posture_service_;
    rclcpp::Publisher<rm_decision_interfaces::msg::SentryPostureStatus>::SharedPtr posture_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::SelfRobotHP>::SharedPtr all_robot_hp_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::FriendLocation>::SharedPtr robot_location_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::RFIDParse>::SharedPtr rfid_pub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::EnemyLocation>::SharedPtr enemy_location_pub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::EnemyForbiddenArea>::SharedPtr enemy_forbidden_area_sub;
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotAreaStatus>::SharedPtr robot_area_status_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_save_map_pub_;

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
    rm_decision_interfaces::msg::RobotControl last_robot_control_cmd_;
    rm_sentry_pp::SendRobotCmdData current_cmd_state_;
    rm_sentry_pp::SendRobotPostureData current_robot_posture_state_;

    // Gimbal angle follow data (protected by tx_mtx_)
    float target_gimbal_yaw_angle_ = 0.0f;
    rclcpp::Time last_gimbal_angle_update_;
    float gimbal_big_yaw_angle_ = 0.0f;
    bool follow_gimbal_big_ = false;
    float gimbal_yaw_ = 0.0f;
    bool track_status_ = false;
    double gimbal_big_yaw_angle_state_;
    bool perception_status_ = true;


    // Gimbal path follow - high frequency resampling
    nav_msgs::msg::Path cached_path_;
    std::mutex path_mtx_;
    rclcpp::TimerBase::SharedPtr gimbal_path_timer_;
    rclcpp::Time last_path_time_;
    int gimbal_path_timeout_ms_ { 1000 };
    double gimbal_yaw_filtered_ { 0.0 };
    size_t prev_nearest_idx_ { 0 };

    // Drift correction timer
    rclcpp::TimerBase::SharedPtr drift_timer_;
    double latest_imu_raw_yaw_ = 0.0;

    // IMU drift correction (protected by tx_mtx_)
    double imu_yaw_offset_ = 0.0;
    bool is_calibrating_imu_ = false;
    rclcpp::Time last_imu_calibration_time_;
    static constexpr double IMU_CALIBRATION_THRESHOLD = 0.05;  // 保留，gimbal_yaw 校准用
    static constexpr double IMU_CALIBRATION_INTERVAL = 2.0;

    // Gimbal big drift correction (protected by tx_mtx_)
    double gimbal_big_drift_ = 0.0;
    double gimbal_big_drift_rate_ = 0.0;
    rclcpp::Time last_drift_update_;
    float gimbal_big_yaw_;

    // Drift calibration state
    bool has_imu_centering_ref_ = false;
    double imu_at_centering_ = 0.0;
    double odom_at_centering_  = 0.0;
    bool odom_stable_ = false;
    int odom_stable_count_ = 0;
    static constexpr int ODOM_STABLE_REQUIRED = 50;  // 50帧 (~5s@10Hz)
    static constexpr double DRIFT_FILTER_ALPHA = 0.15;

    bool tx_pending_ { false };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // chiral
    std::unique_ptr<talos::chiral::navigation::NavigationEndpoint> chiral_reader_;
    rm_decision_interfaces::msg::EnemyForbiddenArea enemy_forbidden_area_;
    bool invincible_cache_[static_cast<size_t>(talos::chiral::navigation::ArmorName::MaxNum)] = {};
    std::mutex invincible_mtx_;
    


    // Odometry cache
    std::mutex odom_mtx_;
    double cached_odom_x_ { 0.0 };
    double cached_odom_y_ { 0.0 };
    double cached_odom_yaw_ { 0.0 };
    double cached_odom_vx_ { 0.0 };
    double cached_odom_vy_ { 0.0 };
    rclcpp::Time last_odom_time_;

    // map→odom cache (relocalization mode only)
    std::mutex map_odom_mtx_;
    double cached_map_to_odom_x_ { 0.0 };
    double cached_map_to_odom_y_ { 0.0 };
    double cached_map_to_odom_yaw_ { 0.0 };
    bool has_cached_map_to_odom_ { false };
    rclcpp::TimerBase::SharedPtr map_odom_timer_;

    // Posture confirmation (service blocks until lower-level reports match)
    std::atomic<uint8_t> pending_confirm_posture_ {0};
    std::atomic<bool> posture_confirmed_ {false};
    int posture_confirm_timeout_ms_ {500};
    int current_posture_ {0};

    // Confidence decay parameters
    double confidence_decay_lambda_ = 0.5;
    double min_confidence_threshold_ = 0.3;

    rm_decision_interfaces::msg::RobotAreaStatus::SharedPtr robot_area_status_;
};

} // namespace rm_sentry_pp_nocrc_serial
