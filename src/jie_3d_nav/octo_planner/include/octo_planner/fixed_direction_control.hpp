#ifndef OCTO_PLANNER__FIXED_DIRECTION_CONTROL_HPP_
#define OCTO_PLANNER__FIXED_DIRECTION_CONTROL_HPP_

#include <algorithm>
#include <cmath>

namespace octo_planner
{

struct Vector2
{
  double x{0.0};
  double y{0.0};
};

inline double norm(const Vector2 & vector)
{
  return std::hypot(vector.x, vector.y);
}

inline bool normalize(const Vector2 & vector, Vector2 & unit_vector)
{
  constexpr double epsilon = 1.0e-9;
  const double length = norm(vector);
  if (length <= epsilon) {
    unit_vector = Vector2{};
    return false;
  }

  unit_vector.x = vector.x / length;
  unit_vector.y = vector.y / length;
  return true;
}

inline Vector2 mapDirectionToBase(const Vector2 & map_direction, double robot_yaw)
{
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  return Vector2{
    cos_yaw * map_direction.x + sin_yaw * map_direction.y,
    -sin_yaw * map_direction.x + cos_yaw * map_direction.y};
}

inline double fixedDirectionSpeed(double distance, double max_speed, double speed_gain)
{
  return std::min(
    std::max(0.0, max_speed),
    std::max(0.0, speed_gain) * std::max(0.0, distance));
}

inline double remainingAlongDirection(
  const Vector2 & target_error_map,
  const Vector2 & fixed_direction_map)
{
  return
    target_error_map.x * fixed_direction_map.x +
    target_error_map.y * fixed_direction_map.y;
}

inline bool shouldSwitchToNearGoal(
  const Vector2 & target_error_map,
  const Vector2 & fixed_direction_map,
  double switch_distance)
{
  const double tolerance = std::max(0.0, switch_distance);
  return
    norm(target_error_map) <= tolerance ||
    remainingAlongDirection(target_error_map, fixed_direction_map) <= tolerance;
}

}  // namespace octo_planner

#endif  // OCTO_PLANNER__FIXED_DIRECTION_CONTROL_HPP_
