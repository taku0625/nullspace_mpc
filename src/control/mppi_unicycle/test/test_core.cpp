#include "mppi_unicycle/core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>
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

// A drive that responds like the Misachiya chassis. The host limits the acceleration of both
// wheels by one ratio, the ESC of each wheel limits it again, and the wheel speed lags behind.
// Localization delivers each pose late and at half the rate of the control loop.
struct SlowDrive
{
  static constexpr double host_accel = 0.264;  // m/s^2 of the wheel tangential speed, 28 RPM/s
  static constexpr double lag = 0.15;
  static constexpr double transport = 0.05;  // From a sent command to the host input.
  static constexpr double compute = 0.01;  // From the start of a solve to the sent command.
  static constexpr double command_timeout = 0.30;
  static constexpr double pose_period = 0.10;
  static constexpr double pose_latency = 0.05;
  static constexpr double half_track = 0.14;

  struct Observation
  {
    double time;
    Pose pose;
    VOmega velocity;
  };

  // The ESCs take one limit for a wheel that speeds up and one for a wheel that slows down.
  SlowDrive(const Pose & start, double esc_accel, double esc_decel)
  : esc_accel(esc_accel), esc_decel(esc_decel), pose(start), observations{{0.0, start, {}}}
  {
  }

  VOmega velocity() const
  {
    return {0.5 * (wheel[0] + wheel[1]), (wheel[1] - wheel[0]) / (2.0 * half_track)};
  }

  void send(const VOmega & command, double at)
  {
    sent.push_back({at, command});
  }

  void step(double dt)
  {
    while (sent.size() > 1 && sent[1].first + transport <= time) {
      sent.pop_front();
    }
    VOmega command;
    if (!sent.empty() && sent.front().first + transport <= time &&
      time < sent.front().first + transport + command_timeout)
    {
      command = sent.front().second;
    }
    const std::array<double, 2> target{
      command.v - half_track * command.omega, command.v + half_track * command.omega};
    const double slew = std::max({1.0, std::abs(target[0] - host[0]) / (host_accel * dt),
        std::abs(target[1] - host[1]) / (host_accel * dt)});
    for (std::size_t i = 0; i < target.size(); ++i) {
      host[i] += (target[i] - host[i]) / slew;
      const double limit = std::abs(host[i]) < std::abs(reference[i]) ? esc_decel : esc_accel;
      reference[i] += std::clamp(host[i] - reference[i], -limit * dt, limit * dt);
      wheel[i] += (reference[i] - wheel[i]) * (1.0 - std::exp(-dt / lag));
    }
    const auto motion = velocity();
    const double heading = pose.yaw + 0.5 * dt * motion.omega;
    pose = {pose.x + dt * motion.v * std::cos(heading), pose.y + dt * motion.v * std::sin(heading),
      wrap(pose.yaw + dt * motion.omega)};
    time += dt;
    if (time + 1e-9 >= observations.back().time + pose_period) {
      observations.push_back({time, pose, motion});
      if (observations.size() > 8) {
        observations.pop_front();
      }
    }
  }

  // The newest pose that localization has delivered by now.
  Observation observed() const
  {
    auto latest = observations.front();
    for (const auto & observation : observations) {
      if (observation.time + pose_latency <= time + 1e-9) {
        latest = observation;
      }
    }
    return latest;
  }

  double esc_accel, esc_decel;
  double time{};
  Pose pose;
  std::array<double, 2> host{}, reference{}, wheel{};
  std::deque<std::pair<double, VOmega>> sent;
  std::deque<Observation> observations;
};

struct Weave
{
  bool completed{};
  double heading_rms{};
  double yaw_rate_rms{};
  double command_lead{};  // Largest difference between the commanded and the actual yaw rate.
  int stops{};  // Times the controller stopped the robot to turn in place.
};

// Follows a straight route for 26 s from 0.1 m beside it and measures the motion after 6 s.
Weave follow_straight_route(
  Settings settings, double esc_accel, bool compensate, double esc_decel = 0.0)
{
  settings.wheel_speed_limit = 0.2827;
  MppiCore controller(settings);
  std::vector<Pose> route;
  for (int i = 0; i <= 200; ++i) {
    route.push_back({0.04 * i, 0.0, 0.0});
  }
  controller.set_route(route);
  SlowDrive drive({0.0, 0.10, 0.0}, esc_accel, esc_decel > 0.0 ? esc_decel : esc_accel);
  Weave weave;
  double heading = 0.0, yaw_rate = 0.0;
  int measured = 0;
  bool moving = false;
  for (int tick = 0; tick < 520; ++tick) {
    const auto observed = drive.observed();
    const double command_time = drive.time + SlowDrive::compute;
    const auto result = compensate ?
      controller.solve(observed.pose, observed.velocity, {}, Timing{observed.time, command_time}) :
      controller.solve(observed.pose, observed.velocity, {});
    if (!result || result->goal_reached) {
      return weave;
    }
    const auto & command = result->command;
    controller.command_sent(command, command_time);
    drive.send(command, command_time);
    weave.stops += moving && command.v == 0.0;
    moving = command.v > 0.0;
    for (int i = 0; i < 50; ++i) {
      drive.step(0.001);
      if (drive.time >= 6.0) {
        const double actual = drive.velocity().omega;
        heading += drive.pose.yaw * drive.pose.yaw;
        yaw_rate += actual * actual;
        weave.command_lead = std::max(weave.command_lead, std::abs(command.omega - actual));
        ++measured;
      }
    }
  }
  weave.completed = true;
  weave.heading_rms = std::sqrt(heading / measured);
  weave.yaw_rate_rms = std::sqrt(yaw_rate / measured);
  return weave;
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

  invalid = Settings{};
  invalid.actuator_dead_time = -0.1;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = Settings{};
  invalid.wheel_accel_limit = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = Settings{};
  invalid.wheel_decel_limit = -0.2;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);
  invalid = Settings{};
  invalid.command_timeout = 0.0;
  EXPECT_THROW(MppiCore{invalid}, std::invalid_argument);

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

TEST(UnicycleMppi, PlansAsBeforeWithoutAnActuatorResponseOrTiming)
{
  Settings settings;
  settings.random_seed = 5;
  Settings slow = settings;
  slow.actuator_dead_time = 0.1;
  slow.actuator_time_constant = 0.2;
  slow.wheel_accel_limit = 0.1;
  // The first plans from the pose. The second is told the times of a current pose and has no
  // response to replay. The third has a response and is not told the times.
  MppiCore plain(settings), timed(settings), untimed(slow);
  const std::vector<Pose> route{{}, {0.5, 0.0, 0.0}, {1.0, 0.3, 0.6}};
  plain.set_route(route);
  timed.set_route(route);
  untimed.set_route(route);
  Pose pose;
  VOmega command;
  for (int step = 0; step < 40; ++step) {
    const double time = control_interval * step;
    const auto a = plain.solve(pose, command, {});
    const auto b = timed.solve(pose, command, {}, Timing{time, time});
    const auto c = untimed.solve(pose, command, {});
    ASSERT_TRUE(a && b && c);
    EXPECT_EQ(a->command.v, b->command.v);
    EXPECT_EQ(a->command.omega, b->command.omega);
    EXPECT_EQ(a->command.v, c->command.v);
    EXPECT_EQ(a->command.omega, c->command.omega);
    EXPECT_EQ(timed.optimal_trajectory().front().x, pose.x);
    command = a->command;
    timed.command_sent(command, time);
    untimed.command_sent(command, time);
    pose = advance(pose, command, control_interval);
  }
}

TEST(UnicycleMppi, StartsTheRolloutsWhereTheSentCommandsTakeTheRobot)
{
  const std::vector<Pose> route{{}, {3.0, 0.0, 0.0}};
  // Sends one command every control interval from time zero, then solves at the given time with
  // a pose measured 0.1 s earlier at the origin. Returns where the rollouts start.
  const auto predicted_start = [&](
    const Settings & settings, const VOmega & command, int commands, double command_time) {
      MppiCore controller(settings);
      controller.set_route(route);
      for (int i = 0; i < commands; ++i) {
        controller.command_sent(command, control_interval * i);
      }
      EXPECT_TRUE(controller.solve({}, command, {}, Timing{command_time - 0.1, command_time}));
      const auto & predicted = controller.optimal_trajectory();
      EXPECT_FALSE(predicted.empty());
      return predicted.empty() ? Pose{} : predicted.front();
    };

  // A dead time of 0.2 s: the command of the solve acts 0.3 s after the pose was measured, and
  // the robot follows the arc of the commands sent before for that time.
  Settings delayed;
  delayed.actuator_dead_time = 0.2;
  const VOmega turning{0.15, 0.3};
  const double angle = turning.omega * 0.3, radius = turning.v / turning.omega;
  auto start = predicted_start(delayed, turning, 21, 1.05);
  EXPECT_NEAR(start.x, radius * std::sin(angle), 1e-5);
  EXPECT_NEAR(start.y, radius * (1.0 - std::cos(angle)), 1e-5);
  EXPECT_NEAR(start.yaw, angle, 1e-9);

  // Wheels that have settled on the command follow the same arc.
  Settings slow = delayed;
  slow.actuator_time_constant = 0.15;
  slow.wheel_accel_limit = 0.264;
  start = predicted_start(slow, turning, 121, 6.05);
  EXPECT_NEAR(start.x, radius * std::sin(angle), 1e-5);
  EXPECT_NEAR(start.y, radius * (1.0 - std::cos(angle)), 1e-5);
  EXPECT_NEAR(start.yaw, angle, 1e-9);

  // From rest the first command reaches the wheels at 0.2 s. With an acceleration limit of
  // 0.1 m/s^2 the speed is 0.1 (t - 0.2) between the pose at 0.95 s and the start at 1.25 s.
  const VOmega straight{0.2, 0.0};
  Settings ramping = delayed;
  ramping.wheel_accel_limit = 0.1;
  start = predicted_start(ramping, straight, 21, 1.05);
  EXPECT_NEAR(start.x, 0.05 * (1.05 * 1.05 - 0.75 * 0.75), 1e-9);
  EXPECT_NEAR(start.y, 0.0, 1e-12);

  // A wheel slows down within its own limit. The wheels run at `before` until commands of
  // `after` start at 5 s and reach them at 5.2 s.
  Settings braking = ramping;
  braking.wheel_decel_limit = 0.3;
  const auto start_after_change = [&](double before, double after, double command_time) {
      MppiCore controller(braking);
      controller.set_route(route);
      for (int i = 0; i < 100; ++i) {
        controller.command_sent({before, 0.0}, control_interval * i);
      }
      for (int i = 100; control_interval * i < command_time; ++i) {
        controller.command_sent({after, 0.0}, control_interval * i);
      }
      const Timing timing{command_time - 0.1, command_time};
      EXPECT_TRUE(controller.solve({}, {before, 0.0}, {}, timing));
      const auto & predicted = controller.optimal_trajectory();
      EXPECT_FALSE(predicted.empty());
      return predicted.empty() ? Pose{} : predicted.front();
    };
  // From 0.2 to 0.1 m/s the speed is 0.2 - 0.3 (t - 5.2). The pose is from 5.15 s and the
  // rollouts start at 5.45 s. Speeding up still takes the limit of 0.1 m/s^2.
  start = start_after_change(0.2, 0.1, 5.25);
  EXPECT_NEAR(start.x, 0.2 * 0.05 + 0.2 * 0.25 - 0.5 * 0.3 * 0.25 * 0.25, 1e-9);
  start = predicted_start(braking, straight, 21, 1.05);
  EXPECT_NEAR(start.x, 0.05 * (1.05 * 1.05 - 0.75 * 0.75), 1e-9);
  // A wheel that reverses slows down to rest at 0.3 m/s^2, which it reaches a third of a
  // second after 5.2 s, and speeds up backward at 0.1 m/s^2 from there.
  start = start_after_change(0.1, -0.1, 6.05);
  const double from = 5.95 - (5.2 + 0.1 / 0.3), to = from + 0.3;
  EXPECT_NEAR(start.x, -0.05 * (to * to - from * from), 1e-9);

  // With a time constant of 0.15 s the speed is 0.2 (1 - exp(-(t - 0.2) / 0.15)) instead,
  // here between the pose at 0.75 s and the start at 1.05 s.
  Settings lagging = delayed;
  lagging.actuator_time_constant = 0.15;
  start = predicted_start(lagging, straight, 17, 0.85);
  EXPECT_NEAR(
    start.x, 0.2 * (0.3 - 0.15 * (std::exp(-0.55 / 0.15) - std::exp(-0.85 / 0.15))), 1e-9);

  // The chassis stops when its commands end, so a later pose is where the robot stays.
  start = predicted_start(delayed, turning, 21, 2.05);
  EXPECT_EQ(start.x, 0.0);
  EXPECT_EQ(start.y, 0.0);
  EXPECT_EQ(start.yaw, 0.0);
}

TEST(UnicycleMppi, KeepsTheSentCommandsAcrossManeuversAndRoutes)
{
  Settings settings;
  settings.actuator_dead_time = 0.2;
  MppiCore controller(settings);
  const std::vector<Pose> route{{}, {3.0, 0.0, 0.0}};
  controller.set_route(route);
  const VOmega cruising{0.15, 0.0};
  for (int i = 0; i <= 20; ++i) {
    controller.command_sent(cruising, control_interval * i);
  }

  // A heading far off the route makes the controller brake, which clears its plan.
  const auto braking = controller.solve({0.0, 0.0, 1.5}, cruising, {}, Timing{1.0, 1.05});
  ASSERT_TRUE(braking);
  EXPECT_EQ(braking->command.v, 0.0);
  EXPECT_EQ(braking->command.omega, 0.0);
  EXPECT_TRUE(controller.optimal_trajectory().empty());
  controller.set_route(route);

  // The wheels still run on the commands sent before: 0.25 s at 0.15 m/s from the pose.
  ASSERT_TRUE(controller.solve({}, {}, {}, Timing{1.0, 1.05}));
  ASSERT_FALSE(controller.optimal_trajectory().empty());
  EXPECT_NEAR(controller.optimal_trajectory().front().x, 0.15 * 0.25, 1e-9);
}

TEST(UnicycleMppi, StopsWeavingOnASlowDriveWithItsResponse)
{
  Settings modelled;
  modelled.actuator_dead_time = SlowDrive::transport;
  modelled.actuator_time_constant = SlowDrive::lag;
  modelled.wheel_accel_limit = SlowDrive::host_accel;
  for (const std::uint32_t seed : {0u, 1u, 2u}) {
    Settings plain;
    plain.random_seed = modelled.random_seed = seed;
    // Planning as if the wheels followed a command at once swings the heading back and forth.
    const auto weaving = follow_straight_route(plain, SlowDrive::host_accel, false);
    ASSERT_TRUE(weaving.completed);
    EXPECT_GT(weaving.heading_rms, 0.08);
    EXPECT_GT(weaving.yaw_rate_rms, 0.20);

    const auto steady = follow_straight_route(modelled, SlowDrive::host_accel, true);
    ASSERT_TRUE(steady.completed);
    EXPECT_LT(steady.heading_rms, 0.04);
    EXPECT_LT(steady.yaw_rate_rms, 0.10);
    EXPECT_LT(3.0 * steady.heading_rms, weaving.heading_rms);
    EXPECT_EQ(steady.stops, 0);
    // The commands do not run ahead of what the wheels deliver.
    EXPECT_LT(steady.command_lead, 0.25);
  }

  // A response that is 30 % off in every part still steadies the heading.
  for (const double scale : {0.7, 1.3}) {
    Settings mismatched = modelled;
    mismatched.random_seed = 0;
    mismatched.actuator_dead_time *= scale;
    mismatched.actuator_time_constant *= scale;
    mismatched.wheel_accel_limit *= scale;
    const auto steady = follow_straight_route(mismatched, SlowDrive::host_accel, true);
    ASSERT_TRUE(steady.completed);
    EXPECT_LT(steady.heading_rms, 0.04);
    EXPECT_LT(steady.yaw_rate_rms, 0.10);
    EXPECT_EQ(steady.stops, 0);
  }
}

TEST(UnicycleMppi, FollowsAnEscWithATighterAccelerationLimit)
{
  const double esc_accel = 0.094;  // 10 RPM/s
  Settings plain;
  // Without the response the heading swings past 45 degrees and the robot stops to turn back.
  const auto weaving = follow_straight_route(plain, esc_accel, false);
  ASSERT_TRUE(weaving.completed);
  EXPECT_GT(weaving.heading_rms, 0.25);
  EXPECT_GT(weaving.stops, 0);

  Settings modelled;
  modelled.actuator_dead_time = SlowDrive::transport;
  modelled.actuator_time_constant = SlowDrive::lag;
  modelled.wheel_accel_limit = esc_accel;
  const auto steady = follow_straight_route(modelled, esc_accel, true);
  ASSERT_TRUE(steady.completed);
  EXPECT_LT(steady.heading_rms, 0.04);
  EXPECT_LT(steady.yaw_rate_rms, 0.10);
  EXPECT_EQ(steady.stops, 0);

  // A limit taken from the host, almost three times too high, still removes most of the swing.
  modelled.wheel_accel_limit = SlowDrive::host_accel;
  const auto optimistic = follow_straight_route(modelled, esc_accel, true);
  ASSERT_TRUE(optimistic.completed);
  EXPECT_LT(3.0 * optimistic.heading_rms, weaving.heading_rms);
  EXPECT_EQ(optimistic.stops, 0);
}

TEST(UnicycleMppi, FollowsAnEscThatLimitsOnlyTheAcceleration)
{
  // The ESCs of the robot speed a wheel up at 10 RPM/s and let it slow down as fast as the
  // host asks, which is 28 RPM/s.
  const double esc_accel = 0.094, esc_decel = 10.0;
  Settings plain;
  const auto weaving = follow_straight_route(plain, esc_accel, false, esc_decel);
  ASSERT_TRUE(weaving.completed);
  EXPECT_GT(weaving.heading_rms, 0.08);

  Settings modelled;
  modelled.actuator_dead_time = SlowDrive::transport;
  modelled.actuator_time_constant = SlowDrive::lag;
  modelled.wheel_accel_limit = esc_accel;
  modelled.wheel_decel_limit = SlowDrive::host_accel;
  const auto steady = follow_straight_route(modelled, esc_accel, true, esc_decel);
  ASSERT_TRUE(steady.completed);
  EXPECT_LT(steady.heading_rms, 0.04);
  EXPECT_LT(steady.yaw_rate_rms, 0.10);
  EXPECT_LT(3.0 * steady.heading_rms, weaving.heading_rms);
  EXPECT_EQ(steady.stops, 0);
}
}  // namespace
}  // namespace mppi_unicycle
