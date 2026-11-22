#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
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
  int samples{1000};
  int prediction_steps{30};
  double prediction_step{0.05};
  std::array<double, 2> sigma{0.5, 0.5};
  double lambda{200.0};
  double max_linear_speed{2.0};
  double max_angular_speed{1.57};
  double reference_speed{1.2};
  double track_width{0.28};
  double wheel_speed_limit{0.0};  // Maximum wheel tangential speed in m/s. Zero leaves it disabled.
  double xy_goal_tolerance{0.15};
  double yaw_goal_tolerance{0.15};
  double velocity_weight{12.0};
  double heading_weight{30.0};
  double distance_weight{35.0};
  double collision_weight{65.0};
  double terminal_weight{60.0};
  std::array<double, 2> control_change_weight{1.0, 1.0};
  std::uint32_t random_seed{0};
};

struct MapCost
{
  double collision{};
  std::optional<double> distance;
  std::optional<double> yaw;
};

using Costmap = std::function<MapCost(const Pose &)>;

struct Solution
{
  VOmega command;
  bool goal_reached{};
};

double wrap(double angle);
double distance(const Pose & a, const Pose & b);
Pose advance(const Pose & pose, const VOmega & input, double dt);

class MppiCore
{
public:
  explicit MppiCore(Settings settings);

  void set_route(std::vector<Pose> route);
  std::optional<Solution> solve(
    const Pose & pose, const VOmega & previous, const Costmap & costmap);

private:
  struct Reference
  {
    Pose pose;
    double speed;
  };

  Pose pose_at(double arc) const;
  void update_progress(const Pose & pose);
  VOmega limit(VOmega input, bool rotate_only) const;
  std::vector<Reference> references(const Pose & pose, bool rotate_only) const;
  std::vector<VOmega> seed(const Pose & pose, const std::vector<Reference> & refs,
    bool rotate_only) const;
  double rollout(
    Pose pose, VOmega previous, const VOmega * controls, const std::vector<Reference> & refs,
    const Costmap & costmap, bool rotate_only) const;

  Settings settings_;
  std::mt19937 random_;
  std::vector<Pose> route_;
  std::vector<double> arc_;
  double progress_{};
  bool warm_{};
  std::vector<VOmega> nominal_;
  std::vector<VOmega> controls_;
  std::vector<double> costs_;
};
}  // namespace mppi_unicycle
