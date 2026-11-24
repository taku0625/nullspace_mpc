#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace mppi_unicycle
{
struct Pose
{
  double x{};
  double y{};
  double yaw{};
};

struct VOmega
{
  double v{};
  double omega{};
};

struct Settings
{
  int samples{512};
  int prediction_steps{25};
  double prediction_step{0.10};
  double exploration{0.1};  // Share of the samples drawn around zero instead of the last solution.
  double lambda{1.0};
  double alpha{0.975};  // Scales the input cost by lambda * (1 - alpha).
  std::array<double, 2> sigma{0.03, 0.25};
  bool reduce_computation{false};  // Draw the noise once and reuse it in every solve.
  double max_linear_speed{0.20};
  double max_angular_speed{1.0};
  double reference_speed{0.15};
  double track_width{0.28};
  double wheel_speed_limit{0.0};  // Maximum wheel tangential speed in m/s. Zero leaves it disabled.
  // Slowest tangential speed of a turning wheel in m/s. Above zero, every command either stops
  // both wheels or runs both at it or faster. The default is just above the 8 RPM start speed of
  // the sensorless Misachiya ESCs at the 0.09 m wheel radius: they trip their overspeed
  // protection on slower commands. Zero removes the floor.
  double wheel_speed_floor{0.0762};
  double xy_goal_tolerance{0.10};
  double yaw_goal_tolerance{0.15};
  double velocity_weight{110.0};
  double heading_weight{2.0};
  double distance_weight{16.0};
  double collision_weight{65.0};
  double terminal_weight{60.0};
  std::array<double, 2> control_change_weight{0.5, 0.5};
  bool use_sg_filter{true};  // Smooth the command with a Savitzky-Golay filter.
  int sg_filter_half_window_size{4};
  int sg_filter_poly_order{6};
  std::uint32_t random_seed{0};
  // Response of the drive between a sent command and the wheel speeds. A command reaches the
  // wheels after the dead time. The reference of each wheel then follows it within the
  // acceleration limit, and the wheel speed follows the reference with a first-order lag.
  // Zero leaves a part out. With all of them at zero the wheels follow a command at once.
  double actuator_dead_time{0.0};  // s
  double actuator_time_constant{0.0};  // s
  double wheel_accel_limit{0.0};  // m/s^2 of the wheel tangential speed while a wheel speeds up
  // m/s^2 while a wheel slows down. Zero takes the limit for speeding up. An ESC that ramps a
  // wheel up slowly and lets it slow down as fast as commanded needs a larger value here.
  double wheel_decel_limit{0.0};
  // Time in seconds for which the chassis keeps driving on its last command.
  double command_timeout{0.30};
  // Detours around obstacles. With a costmap, the route up to detour_range ahead of the robot
  // is examined. Where it passes through collision cost, the robot follows it at lateral
  // offsets of up to detour_max_offset instead: outside the cost where there is room, through
  // the least cost in a narrow gap, and on the side taken before while both stay open. Zero in
  // either value follows the route as recorded.
  double detour_range{2.5};  // m
  double detour_max_offset{1.0};  // m
};

// Times of one solve in seconds on the clock of command_sent().
struct Timing
{
  double pose_time{};  // When the pose was measured.
  double command_time{};  // When the command of this solve is sent.
};

struct MapCost
{
  double collision{};
  std::optional<double> distance;  // Distance to the reference path. Taken from the route if unset.
};

// The solver evaluates the rollouts in parallel and calls this from several threads at once.
using Costmap = std::function<MapCost(const Pose &)>;

struct Solution
{
  VOmega command;
  bool goal_reached{};
};

double wrap(double angle);
double distance(const Pose & a, const Pose & b);
Pose advance(const Pose & pose, const VOmega & input, double dt);

// Weights that give the value of a least-squares polynomial at the center of 2 * half_window + 1
// equally spaced samples.
std::vector<double> savitzky_golay_coefficients(int half_window, int order);

class MppiCore
{
public:
  explicit MppiCore(Settings settings);

  bool set_wheel_speed_limits(double minimum, double maximum);
  void set_route(std::vector<Pose> route);
  // Records a command sent to the chassis, zero commands included. Times must not decrease.
  void command_sent(const VOmega & command, double time);
  // Plans from the pose as if the wheels followed a command at once.
  std::optional<Solution> solve(
    const Pose & pose, const VOmega & measured, const Costmap & costmap);
  // Plans from the state predicted for the time the command starts to act on the wheels. The
  // recorded commands are replayed through the actuator response from the pose time until then.
  // The route progress, the goal and the maneuvers still follow the measured pose.
  std::optional<Solution> solve(
    const Pose & pose, const VOmega & measured, const Costmap & costmap, const Timing & timing);

  // Predictions of the latest solve(). They are empty when it returned without sampling.
  // The optimal trajectory starts at the pose the rollouts started from, which is the solved
  // pose or the one predicted from it, and continues with one pose per step. When every sample
  // was rejected it holds only that first pose.
  const std::vector<Pose> & optimal_trajectory() const;
  // Collision-free sampled trajectories in order of increasing cost.
  std::vector<std::vector<Pose>> elite_trajectories(std::size_t count) const;
  // Every sampled trajectory, prediction_steps poses per sample, and the cost of each sample.
  // A sample that reaches a lethal cell has an infinite cost.
  const std::vector<Pose> & sampled_states() const;
  const std::vector<double> & sample_costs() const;
  // The stretch of the route ahead of the robot as the latest solve() followed it around
  // obstacles. Empty while the route is followed as recorded.
  std::vector<Pose> detour() const;
  double calculation_time_ms() const;

private:
  struct Segment
  {
    double x, y, dx, dy, length;
  };

  // The route as the cost function sees it during one solve.
  struct Tracking
  {
    bool rotate_only;
    bool goal_in_reach;
    Pose target;  // Route position one horizon ahead. It takes the role of the goal state.
    double lookahead;  // Distance along the route to the point that gives the reference direction.
    std::vector<Segment> window;  // Route segments within reach of the rollouts, in path order.
    bool detoured;  // The window is offset from the recorded route.
  };

  // Tangential speeds of the left and right wheel in m/s.
  struct Wheels
  {
    std::array<double, 2> reference{};  // Speeds the drive is ramping the wheels to.
    std::array<double, 2> speed{};
  };

  // State at the start of the rollouts.
  struct Start
  {
    Pose pose;
    Wheels wheels;
    bool wheels_modelled{};  // The rollouts move the wheels through the actuator response.
  };

  struct SentCommand
  {
    double time;
    VOmega command;
  };

  Pose pose_at(double arc) const;
  void plan_detour(const Costmap & costmap);
  double detour_offset(double arc) const;
  Pose followed_pose(double arc) const;  // The route pose with the detour applied.
  double project(const Pose & pose, double progress) const;
  void update_progress(const Pose & pose);
  VOmega limit(VOmega input, bool rotate_only) const;
  Tracking track(bool rotate_only, double progress) const;
  void sample_noise();
  bool wheels_modelled() const;
  VOmega drive(Wheels & wheels, const VOmega & command, double dt) const;
  std::optional<Start> predict(const Pose & pose, const Timing & timing) const;
  double stage_cost(
    const Pose & state, double forward_speed, const VOmega & input, const VOmega & previous,
    const Tracking & tracking, const Costmap & costmap) const;
  double rollout(
    const Start & start, const VOmega * controls, Pose * states, const Tracking & tracking,
    const Costmap & costmap) const;
  VOmega smooth(const std::vector<VOmega> & controls);
  void reset();
  std::optional<Solution> optimize(
    const Pose & pose, const VOmega & measured, const Costmap & costmap, const Timing * timing);

  Settings settings_;
  std::uint64_t noise_draws_{};
  std::vector<Pose> route_;
  std::vector<double> arc_;
  double progress_{};
  // Lateral offsets of the followed route, left positive, at the arcs
  // (detour_first_ + i) * detour_spacing. Empty while the route is followed as recorded.
  std::vector<double> detour_;
  long detour_first_{};
  // With a wheel speed floor the robot drives forward or turns in place, and brakes in between.
  enum class Maneuver {drive, brake, turn};

  double wheel_min_speed_{};
  Maneuver maneuver_{Maneuver::drive};
  double turn_direction_{};

  // The sample buffers hold one row of prediction_steps entries per sample.
  std::vector<VOmega> noise_;
  std::vector<VOmega> controls_;
  std::vector<Pose> states_;
  std::vector<double> costs_;
  std::vector<VOmega> nominal_;  // Optimal input sequence of the previous solve.
  VOmega last_command_;
  std::vector<Pose> optimal_;
  std::vector<double> filter_coefficients_;
  std::vector<VOmega> filter_history_;  // Past filtered commands, oldest first.
  std::deque<SentCommand> sent_;  // Commands sent to the chassis, oldest first.
  double calculation_time_ms_{};
};
}  // namespace mppi_unicycle
