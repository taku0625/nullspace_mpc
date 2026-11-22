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
