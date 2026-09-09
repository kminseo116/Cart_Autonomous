#pragma once

#include <algorithm>
#include <cmath>

namespace rear_ackermann_controller
{

struct VehicleParams
{
  double rear_track_m{0.5};
  double wheel_radius_m{0.2};
  double max_wheel_speed_mps{0.4};
  double max_yaw_rate_rad_s{0.5};
};

struct WheelSpeeds
{
  double left_mps{};
  double right_mps{};
};

inline WheelSpeeds compute_wheel_speeds(
  const double linear_x_mps, double angular_z_rad_s, const VehicleParams & params)
{
  angular_z_rad_s = std::clamp(
    angular_z_rad_s, -params.max_yaw_rate_rad_s, params.max_yaw_rate_rad_s);
  const double half_track_m = params.rear_track_m / 2.0;
  double left_mps = linear_x_mps - angular_z_rad_s * half_track_m;
  double right_mps = linear_x_mps + angular_z_rad_s * half_track_m;

  const double peak_wheel_speed_mps = std::max(std::abs(left_mps), std::abs(right_mps));
  if (peak_wheel_speed_mps > params.max_wheel_speed_mps) {
    const double scale = params.max_wheel_speed_mps / peak_wheel_speed_mps;
    left_mps *= scale;
    right_mps *= scale;
  }

  return {left_mps, right_mps};
}

}  // namespace rear_ackermann_controller
