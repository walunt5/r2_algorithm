#include <cmath>

#include "gtest/gtest.h"
#include "octo_planner/fixed_direction_control.hpp"

namespace
{

constexpr double kTolerance = 1.0e-9;

TEST(FixedDirectionControl, NormalizesInitialTargetError)
{
  octo_planner::Vector2 direction;
  ASSERT_TRUE(octo_planner::normalize({3.0, 4.0}, direction));
  EXPECT_NEAR(direction.x, 0.6, kTolerance);
  EXPECT_NEAR(direction.y, 0.8, kTolerance);
  EXPECT_NEAR(octo_planner::norm(direction), 1.0, kTolerance);
}

TEST(FixedDirectionControl, PreservesMapDirectionWhenRobotYawChanges)
{
  octo_planner::Vector2 direction_map;
  ASSERT_TRUE(octo_planner::normalize({2.0, -1.0}, direction_map));

  for (const double yaw : {0.0, 0.5, -1.2, M_PI}) {
    const auto direction_base = octo_planner::mapDirectionToBase(direction_map, yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double reconstructed_map_x =
      cos_yaw * direction_base.x - sin_yaw * direction_base.y;
    const double reconstructed_map_y =
      sin_yaw * direction_base.x + cos_yaw * direction_base.y;

    EXPECT_NEAR(reconstructed_map_x, direction_map.x, kTolerance);
    EXPECT_NEAR(reconstructed_map_y, direction_map.y, kTolerance);
    EXPECT_NEAR(octo_planner::norm(direction_base), 1.0, kTolerance);
  }
}

TEST(FixedDirectionControl, UsesCruiseSpeedThenProportionalSlowdown)
{
  EXPECT_NEAR(octo_planner::fixedDirectionSpeed(2.0, 1.2, 2.2), 1.2, kTolerance);
  EXPECT_NEAR(octo_planner::fixedDirectionSpeed(0.4, 1.2, 2.2), 0.88, kTolerance);
  EXPECT_NEAR(octo_planner::fixedDirectionSpeed(0.2, 1.2, 2.2), 0.44, kTolerance);
}

TEST(FixedDirectionControl, SwitchesForNearGoalOrTargetPlaneCrossing)
{
  const octo_planner::Vector2 direction_map{1.0, 0.0};

  EXPECT_TRUE(octo_planner::shouldSwitchToNearGoal({0.12, 0.12}, direction_map, 0.2));
  EXPECT_FALSE(octo_planner::shouldSwitchToNearGoal({0.30, 0.50}, direction_map, 0.2));
  EXPECT_TRUE(octo_planner::shouldSwitchToNearGoal({0.19, 0.50}, direction_map, 0.2));
  EXPECT_TRUE(octo_planner::shouldSwitchToNearGoal({-0.10, 0.50}, direction_map, 0.2));
}

}  // namespace
