#include "gtest/gtest.h"
#include "octo_planner/axis_sequential_control.hpp"

namespace
{

using octo_planner::AxisSequentialPhase;

TEST(AxisSequentialControl, CommandsOnlyXWithSignedClampedVelocity)
{
  auto output = octo_planner::updateAxisSequentialControl(
    AxisSequentialPhase::X_ONLY, 2.0, -3.0, 0.1, 2.0, 3.0, 1.5, 1.2, 0.03, 0.03);
  EXPECT_EQ(output.phase, AxisSequentialPhase::X_ONLY);
  EXPECT_DOUBLE_EQ(output.vx, 1.5);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_FALSE(output.use_final_live_error);

  output = octo_planner::updateAxisSequentialControl(
    AxisSequentialPhase::X_ONLY, -2.0, 3.0, 0.1, 2.0, 3.0, 1.5, 1.2, 0.03, 0.03);
  EXPECT_DOUBLE_EQ(output.vx, -1.5);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
}

TEST(AxisSequentialControl, ReachedXCommandsOnlyYInSameUpdate)
{
  const auto output = octo_planner::updateAxisSequentialControl(
    AxisSequentialPhase::X_ONLY, 0.1, -0.5, 0.1, 2.0, 2.0, 1.5, 1.2, 0.03, 0.03);
  EXPECT_EQ(output.phase, AxisSequentialPhase::Y_ONLY);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, -1.0);
  EXPECT_FALSE(output.use_final_live_error);
}

TEST(AxisSequentialControl, ReachedYTransitionsToFinalWithoutReturningToX)
{
  const auto output = octo_planner::updateAxisSequentialControl(
    AxisSequentialPhase::Y_ONLY, 0.6, -0.1, 0.1, 2.0, 2.0, 1.5, 1.2, 0.03, 0.03);
  EXPECT_EQ(output.phase, AxisSequentialPhase::FINAL_LIVE_ERROR);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
  EXPECT_TRUE(output.use_final_live_error);
}

TEST(AxisSequentialControl, ReachedBothTransitionsDirectlyToFinal)
{
  const auto output = octo_planner::updateAxisSequentialControl(
    AxisSequentialPhase::X_ONLY, 0.05, 0.05, 0.1, 2.0, 2.0, 1.5, 1.2, 0.03, 0.03);
  EXPECT_EQ(output.phase, AxisSequentialPhase::FINAL_LIVE_ERROR);
  EXPECT_TRUE(output.use_final_live_error);
}

TEST(AxisSequentialControl, AppliesDeadbandBeforePublishing)
{
  const auto output = octo_planner::updateAxisSequentialControl(
    AxisSequentialPhase::X_ONLY, 0.2, 1.0, 0.1, 0.1, 2.0, 1.5, 1.2, 0.03, 0.03);
  EXPECT_EQ(output.phase, AxisSequentialPhase::X_ONLY);
  EXPECT_DOUBLE_EQ(output.vx, 0.0);
  EXPECT_DOUBLE_EQ(output.vy, 0.0);
}

}  // namespace
