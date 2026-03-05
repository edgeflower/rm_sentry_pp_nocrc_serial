#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include "rm_sentry_pp_nocrc_serial/packet.hpp"
#include "rm_sentry_pp_nocrc_serial/serial_port.hpp"
#include <rm_decision_interfaces/msg/robot_control.hpp>

using namespace std::chrono_literals;

namespace rm_sentry_pp_nocrc_serial
{

    class Node : public rclcpp::Node
    {
    public:
        explicit Node(const rclcpp::NodeOptions &options)
            : rclcpp::Node("rm_sentry_pp_nocrc_serial", options)
        {
            port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
            baud_ = declare_parameter<int>("baud", 115200);
            imu_frame_ = declare_parameter<std::string>("imu_frame", "gimbal_big");
            cmd_vel_chassis_topic_ = declare_parameter<std::string>("cmd_vel_chassis_topic", "cmd_vel_chassis");
            robot_control_topic_ = declare_parameter<std::string>("robot_control_topic", "robot_control");
            imu_topic_ = declare_parameter<std::string>("imu_topic", "imu");
            send_period_ms_ = declare_parameter<int>("send_period_ms", 5);
            enable_dtr_rts_ = declare_parameter<bool>("enable_dtr_rts", true);

            imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

            cmd_vel_chassis_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                cmd_vel_chassis_topic_, 10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                { onCmd(*msg); });

            robot_control_sub_ = create_subscription<rm_decision_interfaces::msg::RobotControl>(
                robot_control_topic_, 10,
                [this](const rm_decision_interfaces::msg::RobotControl::SharedPtr msg)
                { onRobotControl(*msg); });

            node_start_ = this->now();

            protect_thread_ = std::thread([this]()
                                          { protectLoop(); });
            rx_thread_ = std::thread([this]()
                                     { rxLoop(); });
            tx_thread_ = std::thread([this]()
                                     { txLoop(); });

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
        void onCmd(const geometry_msgs::msg::Twist &msg)
        {
            std::lock_guard<std::mutex> lk(tx_mtx_);
            current_cmd_state_.data.speed_vector.vx = msg.linear.x;
            current_cmd_state_.data.speed_vector.vy = msg.linear.y;
            //current_cmd_state_.data.speed_vector.wz = msg.angular.z;
            //current_cmd_state_.data.gimbal_big.yaw_vel = msg.angular.z * 100.0;
            tx_pending_ = true;
            /*
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "Twist in: vx=%.3f vy=%.3f wz=%.3f | stored: vx=%.3f vy=%.3f wz=%.3f",
                                 msg.linear.x, msg.linear.y, msg.angular.z,
                                 current_cmd_state_.data.speed_vector.vx,
                                 current_cmd_state_.data.speed_vector.vy,
                                 current_cmd_state_.data.speed_vector.wz);
                                 */
        }

        void onRobotControl(const rm_decision_interfaces::msg::RobotControl &msg)
        {
            std::lock_guard<std::mutex> lk(tx_mtx_);
            current_cmd_state_.data.gimbal_big.yaw_vel = msg.gimbal_big_yaw_vel; // 这个要往大了写
            current_cmd_state_.data.speed_vector.wz = msg.chassis_spin_vel;  // 只有0与非0， 非0就会转动，0就不会转动
            tx_pending_ = true;
        }
        void protectLoop()
        {
            while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed))
            {
                if (!is_port_ok_.load(std::memory_order_relaxed))
                {
                    std::lock_guard<std::mutex> lk(port_mtx_);
                    sp_.close();
                    if (sp_.open(port_, baud_))
                    {
                        if (enable_dtr_rts_)
                            sp_.setDtrRts(true);
                        is_port_ok_.store(true, std::memory_order_relaxed);
                        RCLCPP_INFO(get_logger(), "Opened port: %s @ %d", port_.c_str(), baud_);
                    }
                    else
                    {
                        // 不刷屏
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                             "Open failed: %s", port_.c_str());
                    }
                }
                std::this_thread::sleep_for(500ms);
            }
        }

        void rxLoop()
        {
            std::vector<uint8_t> rxbuf;
            rxbuf.reserve(4096);

            uint8_t tmp[256];

            while (rclcpp::ok() && !exit_.load(std::memory_order_relaxed))
            {
                if (!is_port_ok_.load(std::memory_order_relaxed))
                {
                    std::this_thread::sleep_for(50ms);
                    continue;
                }

                int n = 0;
                {
                    std::lock_guard<std::mutex> lk(port_mtx_);
                    n = sp_.readSome(tmp, sizeof(tmp), 20);
                }

                if (n < 0)
                {
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

        void parseFrames(std::vector<uint8_t> &rxbuf)
        {
            // 无 CRC：必须靠 SoF + data_len + EoF 强同步
            while (true)
            {
                if (rxbuf.size() < sizeof(rm_sentry_pp::HeaderFrame))
                    return;

                // 找 SoF
                size_t sof_pos = 0;
                while (sof_pos < rxbuf.size() && rxbuf[sof_pos] != rm_sentry_pp::HeaderFrame::SoF())
                    sof_pos++;
                if (sof_pos > 0)
                {
                    rxbuf.erase(rxbuf.begin(), rxbuf.begin() + sof_pos);
                    if (rxbuf.size() < sizeof(rm_sentry_pp::HeaderFrame))
                        return;
                }

                rm_sentry_pp::HeaderFrame hdr{};
                std::memcpy(&hdr, rxbuf.data(), sizeof(hdr));

                // body = time_stamp(4) + payload(data_len) + eof(1)
                const size_t body_len = 4 + static_cast<size_t>(hdr.data_len) + 1;
                const size_t frame_len = sizeof(rm_sentry_pp::HeaderFrame) + body_len;

                // 防御：无 CRC 时必须限制 data_len
                if (hdr.data_len == 0 || hdr.data_len > 64)
                {
                    rxbuf.erase(rxbuf.begin()); // 丢掉这个 SoF，继续找
                    continue;
                }

                if (rxbuf.size() < frame_len)
                    return;

                // 校验 EoF
                if (rxbuf[frame_len - 1] != rm_sentry_pp::HeaderFrame::EoF())
                {
                    rxbuf.erase(rxbuf.begin());
                    continue;
                }

                // 分发（你当前 pocket 只有 IMU 真正从下位机来）
                if (hdr.id == rm_sentry_pp::ID_IMU)
                {
                    // 强一致性：IMU 的 data_len 必须是 24
                    if (hdr.data_len == sizeof(rm_sentry_pp::ReceiveImuData::data) && frame_len == sizeof(rm_sentry_pp::ReceiveImuData))
                    {
                        auto imu = rm_sentry_pp::fromBytes<rm_sentry_pp::ReceiveImuData>(rxbuf.data());
                        publishImu(imu);
                    }
                }

                rxbuf.erase(rxbuf.begin(), rxbuf.begin() + frame_len);
            }
        }

        void publishImu(const rm_sentry_pp::ReceiveImuData &imu_data)
        {
            sensor_msgs::msg::Imu imu;
            imu.header.stamp = this->now();
            imu.header.frame_id = imu_frame_;

            tf2::Quaternion q;
            q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
            imu.orientation = tf2::toMsg(q);

            imu.angular_velocity.x = imu_data.data.roll_vel;
            imu.angular_velocity.y = imu_data.data.pitch_vel;
            imu.angular_velocity.z = imu_data.data.yaw_vel;

            imu_pub_->publish(imu);
        }

        void txLoop()
        {
            rclcpp::WallRate loop_rate{std::chrono::milliseconds(send_period_ms_)};
            while (rclcpp::ok() && !exit_.load())
            {
                if (!is_port_ok_.load())
                {
                    loop_rate.sleep();
                    continue;
                }

                rm_sentry_pp::SendRobotCmdData pkt{};
                {
                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    rm_sentry_pp::fillHeader(pkt, rm_sentry_pp::ID_ROBOT_CMD);

                    pkt.frame_header.id = rm_sentry_pp::ID_ROBOT_CMD;
                    pkt.time_stamp = nowMs();
                    pkt.data = current_cmd_state_.data; // 统一拷贝最新状态
                    pkt.eof = rm_sentry_pp::HeaderFrame::EoF();
                }
                /*
                RCLCPP_INFO(this->get_logger(),"x: %f, y: %f, z: %f, yaw_vel : %f",
                pkt.data.speed_vector.vx, pkt.data.speed_vector.vy,
                 pkt.data.speed_vector.wz, pkt.data.gimbal_big.yaw_vel);*/

                auto bytes = rm_sentry_pp::toVector(pkt);
                /*
                std::stringstream ss;
                for (auto b : bytes)
                    ss << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                RCLCPP_INFO(this->get_logger(), "TX Raw: %s", ss.str().c_str());*/
                std::lock_guard<std::mutex> lk(port_mtx_);
                if (!sp_.writeAll(bytes.data(), bytes.size()))
                {
                    is_port_ok_.store(false);
                }

                loop_rate.sleep();
            }
        }

    private:
        // params
        std::string port_;
        int baud_{115200};
        std::string imu_frame_;
        std::string cmd_vel_chassis_topic_;
        std::string robot_control_topic_;
        std::string imu_topic_;
        int send_period_ms_{5};
        bool enable_dtr_rts_{true};

        // ros
        rclcpp::Time node_start_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_sub_;
        rclcpp::Subscription<rm_decision_interfaces::msg::RobotControl>::SharedPtr robot_control_sub_;

        // serial
        SerialPort sp_;
        std::mutex port_mtx_;
        std::atomic<bool> is_port_ok_{false};

        // threads
        std::atomic<bool> exit_{false};
        std::thread protect_thread_;
        std::thread rx_thread_;
        std::thread tx_thread_;

        // tx data
        std::mutex tx_mtx_;
        rm_sentry_pp::SendRobotCmdData tx_pkt_{};
        rm_decision_interfaces::msg::RobotControl last_robot_control_cmd_;
        rm_sentry_pp::SendRobotCmdData current_cmd_state_; // 存储最新的底盘和云台期望值
        bool tx_pending_{false};
    };

} // namespace rm_sentry_pp_nocrc_serial

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rm_sentry_pp_nocrc_serial::Node>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
