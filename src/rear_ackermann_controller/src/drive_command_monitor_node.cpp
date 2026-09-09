#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DriveCommandMonitorNode : public rclcpp::Node
{
public:
  DriveCommandMonitorNode()
  : Node("drive_command_monitor")
  {
    wheel_radius_m_ = declare_parameter("wheel_radius_m", 0.1016);
    wheel_track_m_ = declare_parameter("wheel_track_m", 0.51345);
    left_actual_rpm_sign_ = declare_parameter("left_actual_rpm_sign", -1.0);
    right_actual_rpm_sign_ = declare_parameter("right_actual_rpm_sign", 1.0);
    linear_deadband_mps_ = declare_parameter("linear_deadband_mps", 0.005);
    angular_deadband_rad_s_ = declare_parameter("angular_deadband_rad_s", 0.01);
    actual_rpm_deadband_ = declare_parameter("actual_rpm_deadband", 0.2);
    const double report_rate_hz = declare_parameter("report_rate_hz", 2.0);

    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        linear_x_ = message->linear.x;
        angular_z_ = message->angular.z;
        cmd_vel_received_ = true;
        last_cmd_vel_time_ = now();
      });
    left_command_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/rear_left_wheel_speed_cmd", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        left_command_rad_s_ = message->data;
        left_command_received_ = true;
      });
    right_command_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/rear_right_wheel_speed_cmd", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        right_command_rad_s_ = message->data;
        right_command_received_ = true;
      });
    left_actual_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/zlac8015d/left_actual_rpm", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        left_actual_raw_rpm_ = message->data;
        left_actual_received_ = true;
      });
    right_actual_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/zlac8015d/right_actual_rpm", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        right_actual_raw_rpm_ = message->data;
        right_actual_received_ = true;
      });

    status_publisher_ = create_publisher<std_msgs::msg::String>("/drive_direction_status", 10);
    const auto report_period = std::chrono::duration<double>(1.0 / std::max(report_rate_hz, 0.1));
    report_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(report_period),
      [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Drive monitor started (read-only): /cmd_vel, wheel commands, and actual RPM");
  }

private:
  std::string classify(const double linear, const double angular) const
  {
    const bool linear_zero = std::abs(linear) <= linear_deadband_mps_;
    const bool angular_zero = std::abs(angular) <= angular_deadband_rad_s_;
    if (linear_zero && angular_zero) {
      return "정지";
    }
    if (angular > angular_deadband_rad_s_) {
      return linear_zero ? "제자리 좌회전" : "좌회전";
    }
    if (angular < -angular_deadband_rad_s_) {
      return linear_zero ? "제자리 우회전" : "우회전";
    }
    return linear > 0.0 ? "직진" : "후진";
  }

  void report()
  {
    if (!cmd_vel_received_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for /cmd_vel...");
      return;
    }

    const double left_physical_rpm = left_actual_raw_rpm_ * left_actual_rpm_sign_;
    const double right_physical_rpm = right_actual_raw_rpm_ * right_actual_rpm_sign_;
    const double rpm_to_mps = 2.0 * M_PI * wheel_radius_m_ / 60.0;
    const double actual_linear_mps =
      0.5 * (left_physical_rpm + right_physical_rpm) * rpm_to_mps;
    const double actual_angular_rad_s =
      (right_physical_rpm - left_physical_rpm) * rpm_to_mps / wheel_track_m_;
    const bool actual_stopped =
      std::abs(left_physical_rpm) <= actual_rpm_deadband_ &&
      std::abs(right_physical_rpm) <= actual_rpm_deadband_;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "command=" << classify(linear_x_, angular_z_)
           << " v=" << linear_x_ << "m/s w=" << angular_z_ << "rad/s";
    if (left_command_received_ && right_command_received_) {
      stream << " | wheel_cmd L=" << left_command_rad_s_
             << " R=" << right_command_rad_s_ << "rad/s";
    } else {
      stream << " | wheel_cmd=WAITING";
    }
    if (left_actual_received_ && right_actual_received_) {
      stream << " | actual="
             << (actual_stopped ? "STOP" : classify(actual_linear_mps, actual_angular_rad_s))
             << " raw_rpm L=" << left_actual_raw_rpm_
             << " R=" << right_actual_raw_rpm_
             << " normalized_rpm L=" << left_physical_rpm
             << " R=" << right_physical_rpm;
    } else {
      stream << " | actual=WAITING";
    }
    stream << " | cmd_age=" << (now() - last_cmd_vel_time_).seconds() << "s";

    std_msgs::msg::String status;
    status.data = stream.str();
    status_publisher_->publish(status);
    RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
  }

  double wheel_radius_m_{0.1016};
  double wheel_track_m_{0.51345};
  double left_actual_rpm_sign_{-1.0};
  double right_actual_rpm_sign_{1.0};
  double linear_deadband_mps_{0.005};
  double angular_deadband_rad_s_{0.01};
  double actual_rpm_deadband_{0.2};
  double linear_x_{0.0};
  double angular_z_{0.0};
  double left_command_rad_s_{0.0};
  double right_command_rad_s_{0.0};
  double left_actual_raw_rpm_{0.0};
  double right_actual_raw_rpm_{0.0};
  bool cmd_vel_received_{false};
  bool left_command_received_{false};
  bool right_command_received_{false};
  bool left_actual_received_{false};
  bool right_actual_received_{false};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_actual_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_actual_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveCommandMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
