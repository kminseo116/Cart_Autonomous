#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuDegreesNode : public rclcpp::Node
{
public:
  ImuDegreesNode()
  : Node("imu_degrees")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    max_integration_dt_sec_ = declare_parameter<double>("max_integration_dt_sec", 0.2);
    publisher_ = create_publisher<geometry_msgs::msg::Vector3>("/imu/deg", 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {on_imu(*message);});
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    const auto & q = message.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    constexpr double kRadToDeg = 180.0 / M_PI;

    geometry_msgs::msg::Vector3 degrees;
    if (std::isfinite(norm) && norm >= 1.0e-12) {
      // orientation을 제공하는 IMU와도 호환된다.
      const double x = q.x / norm;
      const double y = q.y / norm;
      const double z = q.z / norm;
      const double w = q.w / norm;
      degrees.x = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * kRadToDeg;
      const double pitch_sin = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
      degrees.y = std::asin(pitch_sin) * kRadToDeg;
      degrees.z = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * kRadToDeg;
    } else {
      // MID-360은 quaternion을 채우지 않으므로 평면 카트의 gyro z를 적분한 상대 yaw를 표시한다.
      rclcpp::Time stamp(message.header.stamp);
      if (message.header.stamp.nanosec == 0 && message.header.stamp.sec == 0) {
        stamp = now();
      }
      if (!received_baseline_) {
        previous_stamp_ = stamp;
        received_baseline_ = true;
        return;
      }
      const double dt_sec = (stamp - previous_stamp_).seconds();
      previous_stamp_ = stamp;
      if (!std::isfinite(message.angular_velocity.z) || dt_sec <= 0.0 ||
        dt_sec > max_integration_dt_sec_)
      {
        return;
      }
      yaw_rad_ = std::atan2(
        std::sin(yaw_rad_ + message.angular_velocity.z * dt_sec),
        std::cos(yaw_rad_ + message.angular_velocity.z * dt_sec));
      degrees.z = yaw_rad_ * kRadToDeg;
    }
    publisher_->publish(degrees);
  }

  std::string imu_topic_;
  double max_integration_dt_sec_{}, yaw_rad_{};
  bool received_baseline_{false};
  rclcpp::Time previous_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDegreesNode>());
  rclcpp::shutdown();
  return 0;
}
