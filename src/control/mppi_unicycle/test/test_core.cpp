#include "mppi_unicycle/core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace mppi_unicycle
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double control_interval = 0.05;

// Runs the control loop on an ideal unicycle until the goal is reached or the time is over.
template<typename Observer>
bool drive(
  MppiCore & controller, Pose & pose, double seconds, const Costmap & costmap, Observer && observe)
{
  VOmega command;
  for (double time = 0.0; time < seconds; time += control_interval) {
    const auto result = controller.solve(pose, command, costmap);
    if (!result) {
      return false;
    }
    if (result->goal_reached) {
      return true;
    }
    command = result->command;
    observe(pose, command);
    pose = advance(pose, command, control_interval);
  }
  return false;
}

double route_error(const std::vector<Pose> & route, const Pose & pose)
{
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < route.size(); ++i) {
    const double dx = route[i].x - route[i - 1].x, dy = route[i].y - route[i - 1].y;
    const double x = pose.x - route[i - 1].x, y = pose.y - route[i - 1].y;
    const double along = std::clamp((x * dx + y * dy) / (dx * dx + dy * dy), 0.0, 1.0);
    closest = std::min(closest, std::hypot(x - along * dx, y - along * dy));
  }
  return closest;
}

TEST(UnicycleMppi, AdvancesInBodyHeadingAndWrapsYaw)
{
  const Pose pose{1.0, 2.0, pi / 2.0};
  const auto next = advance(pose, {0.4, pi}, 0.5);
  EXPECT_NEAR(next.x, 1.0, 1e-9);
  EXPECT_NEAR(next.y, 2.2, 1e-9);
  EXPECT_NEAR(std::abs(next.yaw), pi, 1e-9);
}

TEST(UnicycleMppi, ComputesSavitzkyGolayWeights)
{
  const auto quadratic = savitzky_golay_coefficients(2, 2);
  const std::vector<double> expected{-3.0 / 35, 12.0 / 35, 17.0 / 35, 12.0 / 35, -3.0 / 35};
  ASSERT_EQ(quadratic.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(quadratic[i], expected[i], 1e-12);
  }

  // A polynomial through every sample returns the center sample.
  const auto exact = savitzky_golay_coefficients(2, 4);
  EXPECT_NEAR(exact[2], 1.0, 1e-9);
  EXPECT_NEAR(exact[0], 0.0, 1e-9);

  double sum = 0.0;
  for (const double weight : savitzky_golay_coefficients(4, 6)) {
    sum += weight;
  }
  EXPECT_NEAR(sum, 1.0, 1e-9);
  EXPECT_THROW(savitzky_golay_coefficients(0, 0), std::invalid_argument);
  EXPECT_THROW(savitzky_golay_coefficients(2, 5), std::invalid_argument);
}

TEST(UnicycleMppi, FollowsACurvedRouteAndTurnsToTheGoalYaw)
{
  const Settings settings;
  MppiCore controller(settings);
  const std::vector<Pose> route{{0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.4, 0.0, 0.0},
    {0.60, 0.12, 0.60}, {0.80, 0.27, 0.70}, {0.95, 0.39, 0.80}};
  controller.set_route(route);
  Pose pose;
  double worst = 0.0;
  bool within_limits = true;
  EXPECT_TRUE(drive(controller, pose, 20.0, {}, [&](const Pose & at, const VOmega & input) {
      worst = std::max(worst, route_error(route, at));
      within_limits = within_limits && input.v >= 0.0 &&
      input.v <= settings.max_linear_speed &&
      std::abs(input.omega) <= settings.max_angular_speed;
    }));
  EXPECT_TRUE(within_limits);
  EXPECT_LT(worst, 0.06);
  EXPECT_LE(distance(pose, route.back()), settings.xy_goal_tolerance);
  EXPECT_LE(std::abs(wrap(pose.yaw - route.back().yaw)), settings.yaw_goal_tolerance);
}

TEST(UnicycleMppi, TurnsAtACornerRecordedAsATurnInPlace)
{
  MppiCore controller(Settings{});
  std::vector<Pose> route;
  for (int i = 0; i <= 15; ++i) {
    route.push_back({0.04 * i, 0.0, 0.0});
  }
  for (int i = 1; i <= 15; ++i) {
    route.push_back({0.6, 0.04 * i, pi / 2.0});
  }
  controller.set_route(route);
  Pose pose;
  double worst = 0.0;
  EXPECT_TRUE(drive(controller, pose, 25.0, {}, [&](const Pose & at, const VOmega &) {
      worst = std::max(worst, route_error(route, at));
    }));
  EXPECT_LT(worst, 0.09);
  EXPECT_NEAR(pose.x, 0.6, 0.05);
}

TEST(UnicycleMppi, LimitsForwardSpeedAndBothWheels)
{
  Settings settings;
  settings.track_width = 0.30;
  settings.wheel_speed_limit = 0.12;
  settings.wheel_speed_floor = 0.0;
  MppiCore controller(settings);
  controller.set_route({{}, {1.0, 0.3, 0.0}});
  Pose pose;
  double fastest_wheel = 0.0;
  drive(controller, pose, 4.0, {}, [&](const Pose &, const VOmega & input) {
      fastest_wheel = std::max(
        fastest_wheel, input.v + 0.5 * settings.track_width * std::abs(input.omega));
    });
  EXPECT_GT(pose.x, 0.2);
  EXPECT_GT(fastest_wheel, 0.08);
  EXPECT_LE(fastest_wheel, 0.12 + 1e-9);
}

TEST(UnicycleMppi, AlignsAndDrivesWithTheNegotiatedWheelSpeedFloor)
{
  const Settings settings;
  const double floor = 0.0754;
  const double half_track = 0.5 * settings.track_width;
  MppiCore controller(settings);
  EXPECT_FALSE(controller.set_wheel_speed_limits(0.25, 0.30));
  ASSERT_TRUE(controller.set_wheel_speed_limits(floor, 0.28));
  controller.set_route({{}, {1.5, 0.0, 0.0}});
  const auto turn = controller.solve({0.0, 0.0, -1.5}, {}, {});
  ASSERT_TRUE(turn);
  EXPECT_DOUBLE_EQ(turn->command.v, 0.0);
  EXPECT_GE(half_track * turn->command.omega, floor - 1e-9);
  const auto aligning = controller.solve({0.0, 0.0, -0.55}, {}, {});
  ASSERT_TRUE(aligning);
  EXPECT_DOUBLE_EQ(aligning->command.v, 0.0);
  EXPECT_GT(aligning->command.omega, 0.0);

  // Once aligned, both wheels stay between the floor and the limit all the way to the goal.
  Pose pose{0.0, 0.0, -0.15};
  bool supported = true;
  EXPECT_TRUE(drive(controller, pose, 20.0, {}, [&](const Pose &, const VOmega & input) {
      const double turn_speed = half_track * std::abs(input.omega);
      supported = supported && (input.v == 0.0 ?
      input.omega == 0.0 || turn_speed >= floor - 1e-9 :
      input.v - turn_speed >= floor - 1e-9 && input.v + turn_speed <= 0.28 + 1e-9);
    }));
  EXPECT_TRUE(supported);
}

TEST(UnicycleMppi, StartsAndReversesWheelsOnlyFromRestByDefault)
{
  Settings settings;
  settings.wheel_speed_limit = 0.28;
  ASSERT_GT(settings.wheel_speed_floor, 0.0754);  // Above the 8 RPM start speed of the ESCs.
  const double half_track = 0.5 * settings.track_width;
  std::vector<Pose> route;
  for (int i = 0; i <= 15; ++i) {
    route.push_back({0.04 * i, 0.0, 0.0});
  }
  for (int i = 1; i <= 15; ++i) {
    route.push_back({0.6, 0.04 * i, pi / 2.0});
  }

  // The heading errors are corrected while driving, by a turn in place, and by a turn back.
  for (const double heading : {0.0, -0.6, -1.5, 3.0}) {
    MppiCore controller(settings);
    controller.set_route(route);
    Pose pose{0.0, 0.0, heading};
    enum class Motion {stop, turn, drive};
    Motion previous = Motion::stop;
    bool supported = true, together = true;
    int turns = 0;
    VOmega first;
    bool started = false;
    EXPECT_TRUE(drive(controller, pose, 40.0, {}, [&](const Pose &, const VOmega & input) {
        const double turn_speed = half_track * std::abs(input.omega);
        const Motion motion = input.v > 0.0 ? Motion::drive :
        input.omega != 0.0 ? Motion::turn : Motion::stop;
        // A turning wheel never runs slower than the floor.
        supported = supported && (motion == Motion::stop ||
        (motion == Motion::turn ? turn_speed >= settings.wheel_speed_floor - 1e-9 :
        input.v - turn_speed >= settings.wheel_speed_floor - 1e-9));
        // Driving and turning in place are separated by a stop of both wheels.
        together = together && (motion == previous || motion == Motion::stop ||
        previous == Motion::stop);
        turns += motion == Motion::turn && previous != Motion::turn;
        previous = motion;
        if (!started) {
          first = input;
          started = true;
        }
      }));
    EXPECT_TRUE(supported);
    EXPECT_TRUE(together);
    if (std::abs(heading) < pi / 4.0) {
      // Both wheels start forward at the floor or faster with the first command.
      EXPECT_GE(first.v - half_track * std::abs(first.omega), settings.wheel_speed_floor - 1e-9);
      EXPECT_EQ(turns, 1);  // Only the corner needs a turn in place.
    } else {
      EXPECT_EQ(first.v, 0.0);
      EXPECT_GT(first.omega * -heading, 0.0);
      EXPECT_EQ(turns, 2);
    }
    EXPECT_NEAR(pose.x, 0.6, 0.06);
  }

  Settings invalid = settings;
  invalid.wheel_speed_floor = 0.25;  // Faster than max_linear_speed.
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = settings;
  invalid.max_angular_speed = 0.4;  // Too slow to turn in place with both wheels at the floor.
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
}

TEST(UnicycleMppi, CorrectsALargeHeadingErrorWhileDrivingWithoutAFloor)
{
  Settings settings;
  settings.wheel_speed_limit = 0.28;
  settings.wheel_speed_floor = 0.0;
  const double old_floor = 0.0754;
  MppiCore controller(settings);

  // The inner wheel then runs slower than the floor would allow.
  controller.set_route({{}, {1.5, 0.0, 0.0}});
  Pose pose{0.0, 0.0, -0.55};
  bool curved = false;
  double fastest_wheel = 0.0;
  drive(controller, pose, 4.0, {}, [&](const Pose &, const VOmega & input) {
      const double turn_speed = 0.5 * settings.track_width * std::abs(input.omega);
      curved = curved ||
      (input.v > 0.0 && input.omega > 0.0 && input.v - turn_speed < old_floor);
      fastest_wheel = std::max(fastest_wheel, input.v + turn_speed);
    });
  EXPECT_TRUE(curved);
  EXPECT_LE(fastest_wheel, 0.28 + 1e-9);
  EXPECT_GT(pose.x, 0.2);
  EXPECT_LT(std::abs(pose.yaw), 0.2);
}

TEST(UnicycleMppi, AlignsAtGoalAndReturnsZeroAfterReachingIt)
{
  const Settings settings;
  MppiCore controller(settings);
  controller.set_route({{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}});
  Pose pose;
  bool stationary = true;
  double fastest_turn = 0.0;
  EXPECT_TRUE(drive(controller, pose, 10.0, {}, [&](const Pose &, const VOmega & input) {
      stationary = stationary && input.v == 0.0;
      fastest_turn = std::max(fastest_turn, input.omega);
    }));
  EXPECT_TRUE(stationary);
  EXPECT_GT(fastest_turn, 0.2);
  EXPECT_LE(std::abs(wrap(pose.yaw - 1.0)), settings.yaw_goal_tolerance);

  const auto reached = controller.solve({0.0, 0.0, 1.0}, {}, {});
  ASSERT_TRUE(reached);
  EXPECT_TRUE(reached->goal_reached);
  EXPECT_EQ(reached->command.v, 0.0);
  EXPECT_EQ(reached->command.omega, 0.0);
}

TEST(UnicycleMppi, RejectsInvalidSettingsAndAnEmptyOrBlockedRoute)
{
  Settings invalid;
  invalid.sigma[0] = 0.0;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = Settings{};
  invalid.exploration = 1.5;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = Settings{};
  invalid.alpha = -0.1;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = Settings{};
  invalid.sg_filter_half_window_size = invalid.prediction_steps;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid.use_sg_filter = false;
  EXPECT_NO_THROW(MppiCore{invalid});

  MppiCore controller(Settings{});
  EXPECT_FALSE(controller.solve({}, {}, {}));
  controller.set_route({{}, {1.0, 0.0, 0.0}});
  EXPECT_FALSE(controller.solve({}, {}, [](const Pose &) {
      return MapCost{std::numeric_limits<double>::infinity(), {}};
    }));
  EXPECT_TRUE(controller.optimal_trajectory().empty());
}

TEST(UnicycleMppi, KeepsPathOrderAtARevisitedStart)
{
  MppiCore controller(Settings{});
  const Pose start{};
  controller.set_route({start, {1.0, 0.0, 0.0}, {1.0, 0.5, pi}, {0.0, 0.0, pi}});
  const Costmap near_the_route = [](const Pose &) {return MapCost{0.0, 0.02};};
  const auto first = controller.solve(start, {}, near_the_route);
  ASSERT_TRUE(first);
  EXPECT_FALSE(first->goal_reached);

  // The goal lies at the start. The first leg is followed before the way back.
  Pose pose = start;
  EXPECT_FALSE(drive(controller, pose, 3.0, near_the_route, [](const Pose &, const VOmega &) {}));
  EXPECT_GT(pose.x, 0.3);
  EXPECT_LT(std::abs(pose.y), 0.05);
}

TEST(UnicycleMppi, PredictsTrajectoriesForVisualization)
{
  Settings settings;
  settings.wheel_speed_floor = 0.0;  // The robot may stand still in front of the wall.
  MppiCore controller(settings);
  EXPECT_TRUE(controller.optimal_trajectory().empty());
  EXPECT_TRUE(controller.elite_trajectories(10).empty());
  controller.set_route({{}, {1.5, 0.0, 0.0}});

  // The robot points at lethal cells left of the route.
  const Costmap wall = [](const Pose & pose) {
      return MapCost{pose.y > 0.01 ? std::numeric_limits<double>::infinity() : 0.0, {}};
    };
  const Pose pose{0.1, 0.0, 0.5};
  ASSERT_TRUE(controller.solve(pose, {}, wall));
  const auto & optimal = controller.optimal_trajectory();
  ASSERT_EQ(optimal.size(), static_cast<std::size_t>(settings.prediction_steps + 1));
  EXPECT_EQ(optimal.front().x, pose.x);
  EXPECT_EQ(optimal.front().y, pose.y);
  EXPECT_LE(optimal.back().y, 0.01);
  EXPECT_GE(controller.calculation_time_ms(), 0.0);

  const auto elite = controller.elite_trajectories(20);
  ASSERT_EQ(elite.size(), 20u);
  for (const auto & trajectory : elite) {
    ASSERT_EQ(trajectory.size(), static_cast<std::size_t>(settings.prediction_steps));
    for (const auto & state : trajectory) {
      EXPECT_LE(state.y, 0.01);
    }
  }
  // Only the collision-free share of the samples is returned.
  const auto feasible = controller.elite_trajectories(10 * settings.samples).size();
  EXPECT_GT(feasible, 20u);
  EXPECT_LT(feasible, static_cast<std::size_t>(settings.samples));

  // Nothing is predicted once the goal is reached.
  controller.set_route({{}, {0.05, 0.0, 0.0}});
  const auto reached = controller.solve({0.05, 0.0, 0.0}, {}, {});
  ASSERT_TRUE(reached);
  EXPECT_TRUE(reached->goal_reached);
  EXPECT_TRUE(controller.optimal_trajectory().empty());
  EXPECT_TRUE(controller.elite_trajectories(10).empty());
}

TEST(UnicycleMppi, RepeatsTheCommandsForTheSameSeed)
{
  Settings settings;
  settings.random_seed = 7;
  MppiCore first(settings), second(settings);
  settings.random_seed = 8;
  MppiCore other(settings);
  const std::vector<Pose> route{{}, {0.5, 0.0, 0.0}, {1.0, 0.3, 0.6}};
  first.set_route(route);
  second.set_route(route);
  other.set_route(route);
  bool differs = false;
  Pose pose;
  VOmega command;
  for (int step = 0; step < 40; ++step) {
    const auto a = first.solve(pose, command, {});
    const auto b = second.solve(pose, command, {});
    const auto c = other.solve(pose, command, {});
    ASSERT_TRUE(a && b && c);
    EXPECT_EQ(a->command.v, b->command.v);
    EXPECT_EQ(a->command.omega, b->command.omega);
    differs = differs || a->command.omega != c->command.omega;
    command = a->command;
    pose = advance(pose, command, control_interval);
  }
  EXPECT_TRUE(differs);
}
}  // namespace
}  // namespace mppi_unicycle
