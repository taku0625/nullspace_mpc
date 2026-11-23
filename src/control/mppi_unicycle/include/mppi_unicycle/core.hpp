#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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
  std::optional<Solution> solve(
    const Pose & pose, const VOmega & measured, const Costmap & costmap);

  // Predictions of the latest solve(). They are empty when it returned without optimizing.
  // The optimal trajectory starts at the solved pose and continues with one pose per step.
  const std::vector<Pose> & optimal_trajectory() const;
  // Collision-free sampled trajectories in order of increasing cost.
  std::vector<std::vector<Pose>> elite_trajectories(std::size_t count) const;
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
  };

  Pose pose_at(double arc) const;
  void update_progress(const Pose & pose);
  VOmega limit(VOmega input, bool rotate_only) const;
  Tracking track(bool rotate_only) const;
  void sample_noise();
  double stage_cost(
    const Pose & state, const VOmega & input, const VOmega & previous, const Tracking & tracking,
    const Costmap & costmap) const;
  double rollout(
    Pose state, const VOmega * controls, Pose * states, const Tracking & tracking,
    const Costmap & costmap) const;
  VOmega smooth(const std::vector<VOmega> & controls);
  void reset();

  Settings settings_;
  std::uint64_t noise_draws_{};
  std::vector<Pose> route_;
  std::vector<double> arc_;
  double progress_{};
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
  double calculation_time_ms_{};
};
}  // namespace mppi_unicycle
