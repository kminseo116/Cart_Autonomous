#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rear_ackermann_controller/differential_drive.hpp"

namespace
{
using rear_ackermann_controller::VehicleParams;

double json_number(const std::string & text, const std::string & key, const double fallback)
{
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  return std::regex_search(text, match, expression) ? std::stod(match[1].str()) : fallback;
}

VehicleParams load_vehicle_params()
{
  std::filesystem::path config_path = std::filesystem::current_path() / "src" / "params_setting.json";
  if (!std::filesystem::exists(config_path)) {
    config_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
      "rear_ackermann_controller")) / "config" / "params_setting.json";
  }
  std::ifstream input(config_path);
  if (!input) {
    throw std::runtime_error("Cannot read " + config_path.string());
  }
  const std::string json_text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  VehicleParams params;
  params.rear_track_m = json_number(json_text, "rear_track_m", params.rear_track_m);
  params.wheel_radius_m = json_number(json_text, "wheel_radius_m", params.wheel_radius_m);
  params.max_wheel_speed_mps = json_number(json_text, "max_wheel_speed_mps", params.max_wheel_speed_mps);
  params.max_yaw_rate_rad_s = json_number(json_text, "max_yaw_rate_rad_s", params.max_yaw_rate_rad_s);
  if (params.rear_track_m <= 0.0 || params.wheel_radius_m <= 0.0) {
    throw std::runtime_error("rear_track_m and wheel_radius_m must be positive");
  }
  return params;
}
}  // namespace

class RearAckermannNode : public rclcpp::Node
{
public:
  RearAckermannNode()
  : Node("rear_ackermann_node"), params_(load_vehicle_params())
  {
    left_publisher_ = create_publisher<std_msgs::msg::Float64>("/rear_left_wheel_speed_cmd", 10);
    right_publisher_ = create_publisher<std_msgs::msg::Float64>("/rear_right_wheel_speed_cmd", 10);
    alignment_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/alignment_cmd", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {on_alignment(*message);});
    manual_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {on_manual(*message);});
    RCLCPP_INFO(get_logger(), "C++ differential-drive node started; /cmd_vel has manual priority");
  }

private:
  void on_alignment(const geometry_msgs::msg::Twist & message)
  {
    if (std::chrono::steady_clock::now() >= manual_until_) {
      apply_twist(message);
    }
  }

  void on_manual(const geometry_msgs::msg::Twist & message)
  {
    manual_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    apply_twist(message);
  }

  void apply_twist(const geometry_msgs::msg::Twist & message)
  {
    const auto speeds = rear_ackermann_controller::compute_wheel_speeds(
      message.linear.x, message.angular.z, params_);
    std_msgs::msg::Float64 left_message;
    std_msgs::msg::Float64 right_message;
    // Gazebo JointController and real motor drivers both consume wheel angular speed.
    left_message.data = speeds.left_mps / params_.wheel_radius_m;
    right_message.data = speeds.right_mps / params_.wheel_radius_m;
    left_publisher_->publish(left_message);
    right_publisher_->publish(right_message);
  }

  VehicleParams params_;
  std::chrono::steady_clock::time_point manual_until_{};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr alignment_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearAckermannNode>());
  rclcpp::shutdown();
  return 0;
}
