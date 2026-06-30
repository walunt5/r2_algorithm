#ifndef OCTO_PLANNER__AXIS_SEQUENTIAL_CONTROL_HPP_
#define OCTO_PLANNER__AXIS_SEQUENTIAL_CONTROL_HPP_

#include <algorithm>
#include <cmath>

namespace octo_planner
{

enum class AxisSequentialPhase
{
  X_ONLY,
  Y_ONLY,
  FINAL_LIVE_ERROR
};

struct AxisSequentialCommand
{
  AxisSequentialPhase phase{AxisSequentialPhase::X_ONLY};
  double vx{0.0};
  double vy{0.0};
  bool use_final_live_error{false};
};

inline double clampAxisCommand(double value, double limit)
{
  const double absolute_limit = std::max(0.0, std::abs(limit));
  return std::max(-absolute_limit, std::min(absolute_limit, value));
}

inline double applyAxisDeadband(double value, double deadband)
{
  return std::abs(value) < std::max(0.0, deadband) ? 0.0 : value;
}

inline AxisSequentialCommand updateAxisSequentialControl(
  AxisSequentialPhase current_phase,
  double x_error,
  double y_error,
  double reached_tolerance,
  double linear_gain,
  double lateral_gain,
  double max_linear_speed,
  double max_lateral_speed,
  double linear_deadband,
  double lateral_deadband)
{
  const double tolerance = std::max(0.0, reached_tolerance);
  AxisSequentialCommand output;
  output.phase = current_phase;

  if (
    output.phase == AxisSequentialPhase::X_ONLY &&
    std::abs(x_error) <= tolerance)
  {
    output.phase = AxisSequentialPhase::Y_ONLY;
  }

  if (output.phase == AxisSequentialPhase::X_ONLY) {
    output.vx = applyAxisDeadband(
      clampAxisCommand(x_error * linear_gain, max_linear_speed), linear_deadband);
    return output;
  }

  if (
    output.phase == AxisSequentialPhase::Y_ONLY &&
    std::abs(y_error) <= tolerance)
  {
    output.phase = AxisSequentialPhase::FINAL_LIVE_ERROR;
  }

  if (output.phase == AxisSequentialPhase::Y_ONLY) {
    output.vy = applyAxisDeadband(
      clampAxisCommand(y_error * lateral_gain, max_lateral_speed), lateral_deadband);
    return output;
  }

  output.use_final_live_error = true;
  return output;
}

}  // namespace octo_planner

#endif  // OCTO_PLANNER__AXIS_SEQUENTIAL_CONTROL_HPP_
