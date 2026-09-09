#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odometry")
  {
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.0);
    wheel_track_m_ = declare_parameter<double>("wheel_track_m", 0.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", true);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.2);
    imu_roll_rad_ = declare_parameter<double>("imu_roll_rad", 0.0);
    imu_pitch_rad_ = declare_parameter<double>("imu_pitch_rad", 0.0);
    if (wheel_radius_m_ <= 0.0 || wheel_track_m_ <= 0.0) {
      throw std::invalid_argument("wheel_radius_m과 wheel_track_m은 실제 측정값으로 설정해야 합니다");
    }
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/zlac8015d/wheel_joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {on_wheel_state(*message);});
    if (use_imu_yaw_) {
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr message) {on_imu(*message);});
    }
  }

private:
  static double normalize_angle(const double angle_rad)
  {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
  }

  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    const auto & angular_velocity = message.angular_velocity;
    if (!std::isfinite(angular_velocity.x) || !std::isfinite(angular_velocity.y) ||
      !std::isfinite(angular_velocity.z))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU 각속도가 유효하지 않습니다");
      return;
    }

    rclcpp::Time stamp(message.header.stamp);
    if (message.header.stamp.nanosec == 0 && message.header.stamp.sec == 0) {
      stamp = now();
    }

    // MID-360은 orientation quaternion 대신 3축 각속도를 제공한다. IMU 장착 roll/pitch를
    // 적용해 각속도 벡터를 base frame으로 회전한 뒤 z 성분을 적분한다.
    const double sin_roll = std::sin(imu_roll_rad_);
    const double cos_roll = std::cos(imu_roll_rad_);
    const double sin_pitch = std::sin(imu_pitch_rad_);
    const double cos_pitch = std::cos(imu_pitch_rad_);
    const double base_angular_z_rad_s =
      -sin_pitch * angular_velocity.x +
      cos_pitch * sin_roll * angular_velocity.y +
      cos_pitch * cos_roll * angular_velocity.z;

    if (!received_imu_baseline_) {
      previous_imu_stamp_ = stamp;
      received_imu_baseline_ = true;
      last_imu_receive_time_ = now();
      received_imu_ = true;
      return;
    }

    const double dt_sec = (stamp - previous_imu_stamp_).seconds();
    previous_imu_stamp_ = stamp;
    last_imu_receive_time_ = now();
    received_imu_ = true;
    if (dt_sec <= 0.0 || dt_sec > imu_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "IMU timestamp 간격이 비정상적이어서 yaw 적분을 건너뜁니다");
      return;
    }
    imu_yaw_rad_ = normalize_angle(imu_yaw_rad_ + base_angular_z_rad_s * dt_sec);
  }

  bool imu_yaw_is_fresh() const
  {
    return received_imu_ && (now() - last_imu_receive_time_).seconds() <= imu_timeout_sec_;
  }

  void on_wheel_state(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.size() != 2 || message.position.size() != 2 ||
      message.name[0] != "left_wheel_joint" || message.name[1] != "right_wheel_joint")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "wheel_joint_states 형식이 예상과 다릅니다");
      return;
    }
    rclcpp::Time stamp(message.header.stamp);
    if (message.header.stamp.nanosec == 0 && message.header.stamp.sec == 0) {
      stamp = now();
    }
    if (!received_baseline_) {
      previous_left_angle_rad_ = message.position[0];
      previous_right_angle_rad_ = message.position[1];                                                                                        
      previous_stamp_ = stamp;
      received_baseline_ = true;
      return;
    }
    if (use_imu_yaw_ && !imu_yaw_is_fresh()) {
      // IMU가 멈춘 동안 encoder yaw를 누적하지 않아, 복구 시 방향 불연속을 막는다.
      //RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "최신 IMU yaw를 기다리는 중입니다");
      previous_left_angle_rad_ = message.position[0];
      previous_right_angle_rad_ = message.position[1];
      previous_stamp_ = stamp;
      return;
    }
    const double delta_left_m = (message.position[0] - previous_left_angle_rad_) * wheel_radius_m_;
    const double delta_right_m = (message.position[1] - previous_right_angle_rad_) * wheel_radius_m_;
    const double delta_s_m = (delta_left_m + delta_right_m) * 0.5;
    // 위치는 encoder 이동량으로, 방향은 MID-360 gyro z 적분 yaw로 계산한다.
    const double next_yaw_rad = use_imu_yaw_ ? imu_yaw_rad_ :
      normalize_angle(yaw_rad_ + (delta_right_m - delta_left_m) / wheel_track_m_);
    const double delta_yaw_rad = normalize_angle(next_yaw_rad - yaw_rad_);
    const double yaw_mid_rad = yaw_rad_ + delta_yaw_rad * 0.5;
    x_m_ += delta_s_m * std::cos(yaw_mid_rad);
    y_m_ += delta_s_m * std::sin(yaw_mid_rad);
    yaw_rad_ = next_yaw_rad;
    const double dt_sec = (stamp - previous_stamp_).seconds();
    publish_odometry(stamp, dt_sec > 0.0 ? delta_s_m / dt_sec : 0.0, dt_sec > 0.0 ? delta_yaw_rad / dt_sec : 0.0);
    previous_left_angle_rad_ = message.position[0];
    previous_right_angle_rad_ = message.position[1];
    previous_stamp_ = stamp;
  }

  void publish_odometry(const rclcpp::Time & stamp, const double linear_x_mps, const double angular_z_rad_s)
  {
    const double half_yaw = yaw_rad_ * 0.5;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);
    odom.twist.twist.linear.x = linear_x_mps;
    odom.twist.twist.angular.z = angular_z_rad_s;
    odom_publisher_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_m_;
    transform.transform.translation.y = y_m_;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  double wheel_radius_m_{}, wheel_track_m_{};
  std::string odom_frame_, base_frame_;
  bool use_imu_yaw_{};
  std::string imu_topic_;
  double imu_timeout_sec_{}, imu_roll_rad_{}, imu_pitch_rad_{};
  bool received_baseline_{false};
  double previous_left_angle_rad_{}, previous_right_angle_rad_{}, x_m_{}, y_m_{}, yaw_rad_{};
  rclcpp::Time previous_stamp_;
  bool received_imu_{false}, received_imu_baseline_{false};
  double imu_yaw_rad_{};
  rclcpp::Time previous_imu_stamp_, last_imu_receive_time_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
