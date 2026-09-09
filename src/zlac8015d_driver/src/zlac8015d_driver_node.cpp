// 실제 카트에 발행하는 ros2 node

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "zlac8015d_driver/modbus_interface.hpp"

namespace
{
constexpr uint16_t kRegisterControlMode = 0x200D;
constexpr uint16_t kRegisterControlWord = 0x200E;
constexpr uint16_t kRegisterLeftAcceleration = 0x2080;
constexpr uint16_t kRegisterRightAcceleration = 0x2081;
constexpr uint16_t kRegisterLeftDeceleration = 0x2082;
constexpr uint16_t kRegisterRightDeceleration = 0x2083;
constexpr uint16_t kRegisterLeftTargetVelocity = 0x2088;
constexpr uint16_t kRegisterLeftFault = 0x20A5;
constexpr uint16_t kRegisterLeftActualPosition = 0x20A7;
constexpr uint16_t kRegisterLeftActualVelocity = 0x20AB;

constexpr uint16_t kVelocityMode = 0x03;
constexpr uint16_t kControlQuickStop = 0x05;
constexpr uint16_t kControlClearFault = 0x06;
constexpr uint16_t kControlStop = 0x07;
constexpr uint16_t kControlEnable = 0x08;
constexpr double kRadPerSecondToRpm = 60.0 / (2.0 * M_PI);
constexpr int16_t kControllerRpmLimit = 3000;

enum class DriverState
{
  DISCONNECTED,
  READY,
  COMMAND_TIMEOUT,
  COMMUNICATION_FAULT,
  DRIVER_FAULT,
};

std::string state_name(const DriverState state)
{
  switch (state) {
    case DriverState::DISCONNECTED: return "DISCONNECTED";
    case DriverState::READY: return "READY";
    case DriverState::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
    case DriverState::COMMUNICATION_FAULT: return "COMMUNICATION_FAULT";
    case DriverState::DRIVER_FAULT: return "DRIVER_FAULT";
  }
  return "UNKNOWN";
}
}  // namespace

class Zlac8015dDriverNode : public rclcpp::Node
{
public:
  Zlac8015dDriverNode()
  : Node("zlac8015d_driver")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    driver_id_ = declare_parameter<int>("driver_id", 1);
    serial_timeout_ms_ = declare_parameter<int>("serial_timeout_ms", 100);
    max_communication_failures_ = declare_parameter<int>("max_communication_failures", 3);
    gear_ratio_ = declare_parameter<double>("gear_ratio", 1.0);
    left_motor_inverted_ = declare_parameter<bool>("left_motor_inverted", false);
    right_motor_inverted_ = declare_parameter<bool>("right_motor_inverted", false);
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm", 300.0);
    acceleration_time_ms_ = declare_parameter<int>("acceleration_time_ms", 500);
    deceleration_time_ms_ = declare_parameter<int>("deceleration_time_ms", 500);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    fault_poll_rate_hz_ = declare_parameter<double>("fault_poll_rate_hz", 10.0);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 1.0);
    encoder_counts_per_rev_ = declare_parameter<double>("encoder_counts_per_rev", 16384.0);
    left_encoder_sign_ = declare_parameter<int>("left_encoder_sign", -1);
    right_encoder_sign_ = declare_parameter<int>("right_encoder_sign", 1);

    validate_parameters();
    left_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/rear_left_wheel_speed_cmd", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {on_left_command(*message);});
    right_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/rear_right_wheel_speed_cmd", 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {on_right_command(*message);});
    left_actual_rpm_publisher_ = create_publisher<std_msgs::msg::Float64>("/zlac8015d/left_actual_rpm", 10);
    right_actual_rpm_publisher_ = create_publisher<std_msgs::msg::Float64>("/zlac8015d/right_actual_rpm", 10);
    left_encoder_publisher_ = create_publisher<std_msgs::msg::Int64>("/zlac8015d/left_encoder_count", 10);
    right_encoder_publisher_ = create_publisher<std_msgs::msg::Int64>("/zlac8015d/right_encoder_count", 10);
    wheel_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/zlac8015d/wheel_joint_states", 10);
    left_fault_publisher_ = create_publisher<std_msgs::msg::UInt16>("/zlac8015d/left_fault", 10);
    right_fault_publisher_ = create_publisher<std_msgs::msg::UInt16>("/zlac8015d/right_fault", 10);
    connected_publisher_ = create_publisher<std_msgs::msg::Bool>("/zlac8015d/connected", 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>("/zlac8015d/state", 10);
    reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "/zlac8015d/reset_fault",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {reset_fault(*response);});

    control_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {control_step();});
    last_command_time_ = now();
    last_reconnect_time_ = now() - rclcpp::Duration::from_seconds(reconnect_interval_sec_);
    // 모든 비교 시간은 node->now()와 같은 ROS time으로 초기화해야 한다.
    last_status_poll_time_ = now();
    RCLCPP_INFO(get_logger(), "ZLAC8015D driver started; waiting for wheel rad/s commands after safe initialization");
  }

  ~Zlac8015dDriverNode() override
  {
    // 종료 중에는 예외를 전파하지 않고, 통신이 살아 있으면 정지 명령을 최우선으로 보낸다.
    safe_stop();
  }

private:
  void validate_parameters()
  {
    if (driver_id_ < 0 || driver_id_ > 127 || serial_timeout_ms_ <= 0 ||
      max_communication_failures_ <= 0 || gear_ratio_ <= 0.0 || max_motor_rpm_ <= 0.0 ||
      max_motor_rpm_ > kControllerRpmLimit || acceleration_time_ms_ < 0 ||
      acceleration_time_ms_ > 32767 || deceleration_time_ms_ < 0 || deceleration_time_ms_ > 32767 ||
      command_timeout_sec_ <= 0.0 || fault_poll_rate_hz_ <= 0.0 || reconnect_interval_sec_ <= 0.0 ||
      encoder_counts_per_rev_ <= 0.0 || (left_encoder_sign_ != -1 && left_encoder_sign_ != 1) ||
      (right_encoder_sign_ != -1 && right_encoder_sign_ != 1))
    {
      throw std::invalid_argument("ZLAC8015D parameter 범위가 올바르지 않습니다");
    }
  }

  void on_left_command(const std_msgs::msg::Float64 & message)
  {
    pending_left_wheel_rad_s_ = message.data;
    left_command_pending_ = true;
    try_commit_wheel_command_pair();
  }

  void on_right_command(const std_msgs::msg::Float64 & message)
  {
    pending_right_wheel_rad_s_ = message.data;
    right_command_pending_ = true;
    try_commit_wheel_command_pair();
  }

  void try_commit_wheel_command_pair()
  {
    if (!left_command_pending_ || !right_command_pending_) {
      return;
    }

    left_wheel_rad_s_ = pending_left_wheel_rad_s_;
    right_wheel_rad_s_ = pending_right_wheel_rad_s_;
    left_command_pending_ = false;
    right_command_pending_ = false;
    command_received_ = true;
    last_command_time_ = now();
  }

  void clear_wheel_command_pair()
  {
    left_wheel_rad_s_ = 0.0;
    right_wheel_rad_s_ = 0.0;
    pending_left_wheel_rad_s_ = 0.0;
    pending_right_wheel_rad_s_ = 0.0;
    left_command_pending_ = false;
    right_command_pending_ = false;
    command_received_ = false;
  }

  bool initialize_driver()
  {
    std::string error;
    if (!modbus_.connect(serial_port_, baudrate_, 'N', 8, 1, driver_id_, serial_timeout_ms_, error)) {
      report_communication_failure(error);
      return false;
    }
    // 연결 또는 재연결 직후에는 이전 명령을 절대 복구하지 않고 0 RPM부터 보낸다.
    if (!write_target_rpm(0, 0) || !write_control(kControlStop) || !poll_status()) {
      modbus_.disconnect();
      return false;
    }
    if (left_fault_ != 0 || right_fault_ != 0) {
      state_ = DriverState::DRIVER_FAULT;
      RCLCPP_ERROR(get_logger(), "드라이버 fault가 있어 Enable하지 않습니다: left=0x%04X right=0x%04X",
        left_fault_, right_fault_);
      return false;
    }
    const bool configured = write_register(kRegisterControlMode, kVelocityMode) &&
      write_register(kRegisterLeftAcceleration, static_cast<uint16_t>(acceleration_time_ms_)) &&
      write_register(kRegisterRightAcceleration, static_cast<uint16_t>(acceleration_time_ms_)) &&
      write_register(kRegisterLeftDeceleration, static_cast<uint16_t>(deceleration_time_ms_)) &&
      write_register(kRegisterRightDeceleration, static_cast<uint16_t>(deceleration_time_ms_)) &&
      write_target_rpm(0, 0) && write_control(kControlEnable);
    if (!configured) {
      modbus_.disconnect();
      return false;
    }
    communication_failures_ = 0;
    clear_wheel_command_pair();
    state_ = DriverState::READY;
    RCLCPP_INFO(get_logger(), "ZLAC8015D Velocity Mode 초기화 완료; 새 ROS 명령 전까지 0 RPM 유지");
    return true;
  }

  void control_step()
  {
    if (!modbus_.connected()) {
      if ((now() - last_reconnect_time_).seconds() >= reconnect_interval_sec_) {
        last_reconnect_time_ = now();
        state_ = DriverState::DISCONNECTED;
        initialize_driver();
      }
      publish_state();
      return;
    }
    if (state_ == DriverState::DRIVER_FAULT || state_ == DriverState::COMMUNICATION_FAULT) {
      publish_state();
      return;
    }
    const bool timed_out = !command_received_ || (now() - last_command_time_).seconds() > command_timeout_sec_;
    if (timed_out) {
      if (state_ != DriverState::COMMAND_TIMEOUT) {
        RCLCPP_WARN(get_logger(), "wheel command timeout: 0 RPM을 전송합니다");
        clear_wheel_command_pair();
      }
      state_ = DriverState::COMMAND_TIMEOUT;
      write_target_rpm(0, 0);
    } else {
      state_ = DriverState::READY;
      write_target_rpm(to_motor_rpm(left_wheel_rad_s_, left_motor_inverted_),
        to_motor_rpm(right_wheel_rad_s_, right_motor_inverted_));
    }
    const double poll_period_sec = 1.0 / fault_poll_rate_hz_;
    if (modbus_.connected() && (now() - last_status_poll_time_).seconds() >= poll_period_sec) {
      last_status_poll_time_ = now();
      poll_status();
    }
    publish_state();
  }

  int16_t to_motor_rpm(const double wheel_rad_s, const bool inverted) const
  {
    double rpm = wheel_rad_s * kRadPerSecondToRpm * gear_ratio_;
    if (inverted) {
      rpm = -rpm;
    }
    rpm = std::clamp(rpm, -max_motor_rpm_, max_motor_rpm_);
    return static_cast<int16_t>(std::lround(rpm));
  }

  bool write_target_rpm(const int16_t left_rpm, const int16_t right_rpm)
  {
    // I16의 two's complement bit pattern을 유지해 0x10으로 좌/우 목표를 동시에 쓴다.
    const uint16_t targets[2] = {static_cast<uint16_t>(left_rpm), static_cast<uint16_t>(right_rpm)};
    std::string error;
    if (!modbus_.write_registers(kRegisterLeftTargetVelocity, targets, 2, error)) {
      report_communication_failure(error);
      return false;
    }
    communication_failures_ = 0;
    return true;
  }

  bool write_register(const uint16_t address, const uint16_t value)
  {
    std::string error;
    if (!modbus_.write_register(address, value, error)) {
      report_communication_failure(error);
      return false;
    }
    communication_failures_ = 0;
    return true;
  }

  bool write_control(const uint16_t control_word)
  {
    return write_register(kRegisterControlWord, control_word);
  }

  bool poll_status()
  {
    // 0x20A5부터 8개를 한 transaction으로 읽어 좌/우 fault, position, velocity를 같은 샘플로 묶는다.
    uint16_t status[8]{};
    std::string error;
    if (!modbus_.read_registers(kRegisterLeftFault, status, 8, error))
    {
      report_communication_failure(error);
      return false;
    }
    communication_failures_ = 0;
    left_fault_ = status[0];
    right_fault_ = status[1];
    left_fault_publisher_->publish(std_msgs::msg::UInt16().set__data(left_fault_));
    right_fault_publisher_->publish(std_msgs::msg::UInt16().set__data(right_fault_));
    const int32_t left_raw_count = combine_signed_32(status[2], status[3]);
    const int32_t right_raw_count = combine_signed_32(status[4], status[5]);
    publish_encoder_feedback(left_raw_count, right_raw_count);
    left_actual_rpm_publisher_->publish(std_msgs::msg::Float64().set__data(static_cast<int16_t>(status[6]) / 10.0));
    right_actual_rpm_publisher_->publish(std_msgs::msg::Float64().set__data(static_cast<int16_t>(status[7]) / 10.0));
    if (left_fault_ != 0 || right_fault_ != 0) {
      RCLCPP_ERROR(get_logger(), "ZLAC8015D fault 감지: left=0x%04X right=0x%04X; 명령을 차단합니다",
        left_fault_, right_fault_);
      write_target_rpm(0, 0);
      write_control(kControlStop);
      state_ = DriverState::DRIVER_FAULT;
      clear_wheel_command_pair();
    }
    return true;
  }

  static int32_t combine_signed_32(const uint16_t high, const uint16_t low)
  {
    const uint32_t bits = (static_cast<uint32_t>(high) << 16) | static_cast<uint32_t>(low);
    return static_cast<int32_t>(bits);
  }

  static int64_t wrap_aware_delta(const int32_t current, const int32_t previous)
  {
    int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);
    if (delta > std::numeric_limits<int32_t>::max()) {
      delta -= (int64_t{1} << 32);
    } else if (delta < std::numeric_limits<int32_t>::min()) {
      delta += (int64_t{1} << 32);
    }
    return delta;
  }

  void publish_encoder_feedback(const int32_t left_raw_count, const int32_t right_raw_count)
  {
    // count topic은 드라이버 register의 raw I32 값이다. wrap 확장은 이 노드 내부에서만 수행한다.
    left_encoder_publisher_->publish(std_msgs::msg::Int64().set__data(left_raw_count));
    right_encoder_publisher_->publish(std_msgs::msg::Int64().set__data(right_raw_count));
    if (!encoder_baseline_set_) {
      previous_left_raw_count_ = left_raw_count;
      previous_right_raw_count_ = right_raw_count;
      encoder_baseline_set_ = true;
    } else {
      left_accumulated_count_ += wrap_aware_delta(left_raw_count, previous_left_raw_count_);
      right_accumulated_count_ += wrap_aware_delta(right_raw_count, previous_right_raw_count_);
      previous_left_raw_count_ = left_raw_count;
      previous_right_raw_count_ = right_raw_count;
    }
    // JointState position은 ROS 정방향 기준의 연속 wheel angle [rad]이며 두 바퀴를 한 메시지로 동기화한다.
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    joints.name = {"left_wheel_joint", "right_wheel_joint"};
    const double count_to_rad = 2.0 * M_PI / encoder_counts_per_rev_;
    joints.position = {left_encoder_sign_ * left_accumulated_count_ * count_to_rad,
      right_encoder_sign_ * right_accumulated_count_ * count_to_rad};
    wheel_joint_state_publisher_->publish(joints);
  }

  void report_communication_failure(const std::string & error)
  {
    ++communication_failures_;
    RCLCPP_ERROR(get_logger(), "ZLAC8015D 통신 오류 (%d/%d): %s", communication_failures_,
      max_communication_failures_, error.c_str());
    if (communication_failures_ >= max_communication_failures_) {
      // 단절 후에는 이전 비영 명령을 저장하거나 재사용하지 않는다.
      clear_wheel_command_pair();
      state_ = DriverState::COMMUNICATION_FAULT;
      modbus_.disconnect();
    }
  }

  void safe_stop() noexcept
  {
    if (!modbus_.connected()) {
      return;
    }
    try {
      write_target_rpm(0, 0);
      write_control(kControlStop);
      modbus_.disconnect();
    } catch (...) {
      // destructor에서는 어떤 예외도 밖으로 전파하지 않는다.
    }
  }

  void reset_fault(std_srvs::srv::Trigger::Response & response)
  {
    if (!modbus_.connected()) {
      response.success = false;
      response.message = "RS485가 연결되지 않아 fault reset을 수행할 수 없습니다";
      return;
    }
    // 자동 reset은 하지 않는다. 이 service 호출만이 명시적 fault clear 경로다.
    if (!write_target_rpm(0, 0) || !write_control(kControlStop) || !write_control(kControlClearFault)) {
      response.success = false;
      response.message = "fault clear Modbus 명령 전송 실패";
      return;
    }
    clear_wheel_command_pair();
    if (!poll_status() || left_fault_ != 0 || right_fault_ != 0) {
      state_ = DriverState::DRIVER_FAULT;
      response.success = false;
      response.message = "fault가 남아 있어 Enable하지 않았습니다";
      return;
    }
    if (!write_register(kRegisterControlMode, kVelocityMode) || !write_target_rpm(0, 0) ||
      !write_control(kControlEnable))
    {
      response.success = false;
      response.message = "fault clear 후 Velocity Mode 재초기화 실패";
      return;
    }
    state_ = DriverState::READY;
    response.success = true;
    response.message = "fault clear 완료; 새 wheel command 전까지 0 RPM 유지";
  }

  void publish_state()
  {
    connected_publisher_->publish(std_msgs::msg::Bool().set__data(modbus_.connected()));
    state_publisher_->publish(std_msgs::msg::String().set__data(state_name(state_)));
  }

  std::string serial_port_;
  int baudrate_{}, driver_id_{}, serial_timeout_ms_{}, max_communication_failures_{};
  double gear_ratio_{}, max_motor_rpm_{}, command_timeout_sec_{}, fault_poll_rate_hz_{}, reconnect_interval_sec_{};
  double encoder_counts_per_rev_{};
  bool left_motor_inverted_{}, right_motor_inverted_{};
  int left_encoder_sign_{}, right_encoder_sign_{};
  int acceleration_time_ms_{}, deceleration_time_ms_{};
  double left_wheel_rad_s_{}, right_wheel_rad_s_{};
  double pending_left_wheel_rad_s_{}, pending_right_wheel_rad_s_{};
  bool left_command_pending_{false}, right_command_pending_{false};
  bool command_received_{false};
  int communication_failures_{0};
  uint16_t left_fault_{}, right_fault_{};
  bool encoder_baseline_set_{false};
  int32_t previous_left_raw_count_{}, previous_right_raw_count_{};
  int64_t left_accumulated_count_{}, right_accumulated_count_{};
  DriverState state_{DriverState::DISCONNECTED};
  rclcpp::Time last_command_time_, last_reconnect_time_, last_status_poll_time_;
  zlac8015d_driver::ModbusInterface modbus_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_subscription_, right_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_actual_rpm_publisher_, right_actual_rpm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_encoder_publisher_, right_encoder_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr left_fault_publisher_, right_fault_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zlac8015dDriverNode>());
  rclcpp::shutdown();
  return 0;
}
