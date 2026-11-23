#include "mppi_unicycle/core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace mppi_unicycle
{
namespace
{
constexpr double pi = 3.14159265358979323846;

TEST(UnicycleMppi, AdvancesInBodyHeadingAndWrapsYaw)
{
  const Pose pose{1.0, 2.0, pi / 2.0};
  const auto next = advance(pose, {0.4, pi}, 0.5);
  EXPECT_NEAR(next.x, 1.0, 1e-9);
  EXPECT_NEAR(next.y, 2.2, 1e-9);
  EXPECT_NEAR(std::abs(next.yaw), pi, 1e-9);
}

TEST(UnicycleMppi, LimitsForwardSpeedAndBothWheels)
{
  Settings settings;
  settings.samples = 128;
  settings.track_width = 0.30;
  settings.wheel_speed_limit = 0.25;
  MppiCore controller(settings);
  const Pose start{};
  controller.set_route({start, {1.0, 0.3, 0.0}});
  const auto result = controller.solve(start, {}, {});
  ASSERT_TRUE(result);
  const auto input = result->command;
  EXPECT_GT(input.v, 0.0);
  EXPECT_LE(input.v + 0.5 * settings.track_width * std::abs(input.omega), 0.25 + 1e-9);
}

TEST(UnicycleMppi, AlignsAndDrivesWithTheNegotiatedWheelSpeedFloor)
{
  Settings settings;
  settings.samples = 128;
  settings.max_linear_speed = 0.20;
  settings.max_angular_speed = 1.0;
  settings.reference_speed = 0.15;
  const double floor = 0.0754;
  MppiCore controller(settings);
  EXPECT_FALSE(controller.set_wheel_speed_limits(0.25, 0.30));
  ASSERT_TRUE(controller.set_wheel_speed_limits(floor, 0.28));
  controller.set_route({{}, {1.5, 0.0, 0.0}});
  const auto turn = controller.solve({0.0, 0.0, -1.5}, {}, {});
  ASSERT_TRUE(turn);
  EXPECT_DOUBLE_EQ(turn->command.v, 0.0);
  EXPECT_GE(0.5 * settings.track_width * turn->command.omega, floor - 1e-9);
  const auto aligning = controller.solve({0.0, 0.0, -0.55}, {}, {});
  ASSERT_TRUE(aligning);
  EXPECT_DOUBLE_EQ(aligning->command.v, 0.0);
  EXPECT_GT(aligning->command.omega, 0.0);

  const auto drive = controller.solve({0.0, 0.0, -0.15}, {}, {});
  ASSERT_TRUE(drive);
  const double turn_speed = 0.5 * settings.track_width * std::abs(drive->command.omega);
  EXPECT_GE(drive->command.v - turn_speed, floor - 1e-9);
  EXPECT_LE(drive->command.v + turn_speed, 0.28 + 1e-9);
}

TEST(UnicycleMppi, TracksWithoutAFloorWhenTheEscReportsNone)
{
  Settings settings;
  settings.samples = 128;
  settings.max_linear_speed = 0.20;
  settings.max_angular_speed = 1.0;
  settings.reference_speed = 0.15;
  const double old_floor = 0.0754;
  MppiCore controller(settings);
  ASSERT_TRUE(controller.set_wheel_speed_limits(0.0, 0.28));

  // A heading error that needs an in-place alignment with a floor is corrected while driving.
  controller.set_route({{}, {1.5, 0.0, 0.0}});
  const auto curve = controller.solve({0.0, 0.0, -0.55}, {}, {});
  ASSERT_TRUE(curve);
  EXPECT_GT(curve->command.v, 0.0);
  EXPECT_GT(curve->command.omega, 0.0);
  const double turn_speed = 0.5 * settings.track_width * std::abs(curve->command.omega);
  EXPECT_LT(curve->command.v - turn_speed, old_floor);
  EXPECT_LE(curve->command.v + turn_speed, 0.28 + 1e-9);
}

TEST(UnicycleMppi, AlignsAtGoalAndReturnsZeroAfterReachingIt)
{
  Settings settings;
  settings.samples = 128;
  MppiCore controller(settings);
  controller.set_route({{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}});
  const auto turning = controller.solve({}, {}, {});
  ASSERT_TRUE(turning);
  EXPECT_FALSE(turning->goal_reached);
  EXPECT_EQ(turning->command.v, 0.0);
  EXPECT_GT(turning->command.omega, 0.0);

  const auto reached = controller.solve({0.0, 0.0, 1.0}, {}, {});
  ASSERT_TRUE(reached);
  EXPECT_TRUE(reached->goal_reached);
  EXPECT_EQ(reached->command.v, 0.0);
  EXPECT_EQ(reached->command.omega, 0.0);
}

TEST(UnicycleMppi, RejectsInvalidNoiseAndAnEmptyOrBlockedRoute)
{
  Settings invalid;
  invalid.sigma[0] = 0.0;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);

  MppiCore controller(Settings{});
  EXPECT_FALSE(controller.solve({}, {}, {}));
  controller.set_route({{}, {1.0, 0.0, 0.0}});
  EXPECT_FALSE(controller.solve({}, {}, [](const Pose &) {
    return MapCost{std::numeric_limits<double>::infinity(), {}, {}};
  }));
}

TEST(UnicycleMppi, KeepsPathOrderAtARevisitedStart)
{
  Settings settings;
  settings.samples = 128;
  MppiCore controller(settings);
  const Pose start{};
  controller.set_route({start, {1.0, 0.0, 0.0}, {1.0, 0.5, pi}, {0.0, 0.0, pi}});
  const auto result = controller.solve(start, {}, [](const Pose &) {
    return MapCost{0.0, 0.02, pi};
  });
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->goal_reached);
  EXPECT_GT(result->command.v, 0.0);
}
}  // namespace
}  // namespace mppi_unicycle
