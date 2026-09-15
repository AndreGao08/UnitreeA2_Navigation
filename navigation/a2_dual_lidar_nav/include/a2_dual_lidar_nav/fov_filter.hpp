#pragma once

#include <algorithm>
#include <cmath>

namespace a2_dual_lidar_nav
{

constexpr double kPi = 3.14159265358979323846;

inline double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool withinHorizontalFov(
  double x, double y, double center_angle_rad, double horizontal_fov_rad)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  if (horizontal_fov_rad >= 2.0 * kPi) {
    return true;
  }
  const double half_fov = std::max(0.0, horizontal_fov_rad) * 0.5;
  const double relative_angle = normalizeAngle(std::atan2(y, x) - center_angle_rad);
  return std::abs(relative_angle) <= half_fov;
}

}  // namespace a2_dual_lidar_nav
