#include "mppi_unicycle/core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace mppi_unicycle
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double no_path = std::numeric_limits<double>::infinity();

double square(double value)
{
  return value * value;
}

bool finite(const Pose & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}
}  // namespace

double wrap(double angle)
{
  return std::remainder(angle, 2.0 * pi);
}

double distance(const Pose & a, const Pose & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Pose advance(const Pose & pose, const VOmega & input, double dt)
{
  return {
    pose.x + dt * input.v * std::cos(pose.yaw),
    pose.y + dt * input.v * std::sin(pose.yaw),
    wrap(pose.yaw + dt * input.omega),
  };
}

MppiCore::MppiCore(Settings settings)
: settings_(settings), random_(settings.random_seed)
{
  const std::array positive{
    settings.prediction_step, settings.sigma[0], settings.sigma[1], settings.lambda,
    settings.max_linear_speed, settings.max_angular_speed, settings.reference_speed,
    settings.track_width, settings.xy_goal_tolerance, settings.yaw_goal_tolerance,
  };
  const std::array nonnegative{
    settings.velocity_weight, settings.heading_weight, settings.distance_weight,
    settings.collision_weight, settings.terminal_weight, settings.control_change_weight[0],
    settings.control_change_weight[1], settings.wheel_speed_limit,
  };
  if (settings.samples < 3 || settings.prediction_steps < 2 ||
    !std::all_of(positive.begin(), positive.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;}) ||
    !std::all_of(nonnegative.begin(), nonnegative.end(),
      [](double value) {return std::isfinite(value) && value >= 0.0;}))
  {
    throw std::invalid_argument("Invalid MPPI horizon, noise, limits or cost weights");
  }
  nominal_.resize(settings.prediction_steps);
  controls_.resize(static_cast<std::size_t>(settings.samples) * settings.prediction_steps);
  costs_.resize(settings.samples);
}

void MppiCore::set_route(std::vector<Pose> route)
{
  if (!std::all_of(route.begin(), route.end(), finite)) {
    throw std::invalid_argument("The route contains a non-finite pose");
  }
  route_.clear();
  arc_.clear();
  for (const auto & point : route) {
    const double span = route_.empty() ? 0.0 : distance(route_.back(), point);
    if (!route_.empty() && span < 1e-6) {
      route_.back() = point;  // Keep the requested final yaw for a rotation in place.
      continue;
    }
    arc_.push_back(arc_.empty() ? 0.0 : arc_.back() + span);
    route_.push_back(point);
  }
  progress_ = 0.0;
  warm_ = false;
}

Pose MppiCore::pose_at(double arc) const
{
  if (arc >= arc_.back()) {
    return route_.back();
  }
  const auto end = std::upper_bound(arc_.begin(), arc_.end(), arc) - arc_.begin();
  const auto & a = route_[end - 1];
  const auto & b = route_[end];
  const double ratio = (arc - arc_[end - 1]) / (arc_[end] - arc_[end - 1]);
  return {a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y),
    std::atan2(b.y - a.y, b.x - a.x)};
}

void MppiCore::update_progress(const Pose & pose)
{
  if (progress_ >= arc_.back()) {
    return;
  }
  // Bound projection to nearby segments to preserve ordering at path crossings.
  const double end = std::min(arc_.back(), progress_ + std::max(0.4, settings_.max_linear_speed * 0.3));
  auto segment = std::upper_bound(arc_.begin(), arc_.end(), progress_) - arc_.begin();
  double closest_arc = progress_;
  double closest_distance = distance(pose, pose_at(progress_));
  for (; segment < static_cast<decltype(segment)>(arc_.size()) && arc_[segment - 1] < end; ++segment) {
    const auto & a = route_[segment - 1];
    const auto & b = route_[segment];
    const double span = arc_[segment] - arc_[segment - 1];
    const double along = ((pose.x - a.x) * (b.x - a.x) + (pose.y - a.y) * (b.y - a.y)) / span;
    const double projected = std::clamp(
      arc_[segment - 1] + along, std::max(arc_[segment - 1], progress_),
      std::min(arc_[segment], end));
    const double error = distance(pose, pose_at(projected));
    if (error < closest_distance) {
      closest_distance = error;
      closest_arc = projected;
    }
  }
  progress_ = closest_arc;
}

VOmega MppiCore::limit(VOmega input, bool rotate_only) const
{
  input.v = rotate_only ? 0.0 : std::clamp(input.v, 0.0, settings_.max_linear_speed);
  input.omega = std::clamp(input.omega, -settings_.max_angular_speed, settings_.max_angular_speed);
  const double fastest_wheel = input.v + 0.5 * settings_.track_width * std::abs(input.omega);
  if (settings_.wheel_speed_limit > 0.0 && fastest_wheel > settings_.wheel_speed_limit) {
    const double scale = settings_.wheel_speed_limit / fastest_wheel;
    input.v *= scale;
    input.omega *= scale;
  }
  return input;
}

std::vector<MppiCore::Reference> MppiCore::references(const Pose & pose, bool rotate_only) const
{
  std::vector<Reference> result;
  result.reserve(settings_.prediction_steps);
  double arc = progress_;
  const auto & goal = route_.back();
  const double goal_distance = distance(pose, goal);
  for (int step = 0; step < settings_.prediction_steps; ++step) {
    const double remaining = std::max(arc_.back() - arc, goal_distance - (arc - progress_));
    const double speed = rotate_only ? 0.0 :
      std::min({settings_.reference_speed, settings_.max_linear_speed, std::sqrt(1.2 * remaining)});
    arc = std::min(arc + speed * settings_.prediction_step, arc_.back());
    auto target = rotate_only ? goal : pose_at(arc);
    if (!rotate_only && arc >= arc_.back()) {
      target.yaw = std::atan2(goal.y - pose.y, goal.x - pose.x);
    }
    result.push_back({target, speed});
  }
  return result;
}

std::vector<VOmega> MppiCore::seed(
  const Pose & initial, const std::vector<Reference> & refs, bool rotate_only) const
{
  std::vector<VOmega> profile;
  profile.reserve(refs.size());
  Pose pose = initial;
  for (const auto & ref : refs) {
    const double error = distance(pose, ref.pose);
    const double target_yaw = rotate_only || error < settings_.xy_goal_tolerance / 2.0 ?
      ref.pose.yaw : std::atan2(ref.pose.y - pose.y, ref.pose.x - pose.x);
    const double turn = wrap(target_yaw - pose.yaw);
    const VOmega input = limit(
      {ref.speed * std::max(0.0, std::cos(turn)), 3.0 * turn}, rotate_only);
    profile.push_back(input);
    pose = advance(pose, input, settings_.prediction_step);
  }
  return profile;
}

double MppiCore::rollout(
  Pose pose, VOmega previous, const VOmega * controls, const std::vector<Reference> & refs,
  const Costmap & costmap, bool rotate_only, std::vector<Pose> * states) const
{
  if (states) {
    states->push_back(pose);
  }
  double cost = 0.0;
  for (std::size_t step = 0; step < refs.size(); ++step) {
    const auto input = controls[step];
    pose = advance(pose, input, settings_.prediction_step);
    const auto & ref = refs[step];
    const auto map = costmap ? costmap(pose) : MapCost{};
    if (!std::isfinite(map.collision)) {
      return no_path;
    }
    const double yaw = rotate_only ? ref.pose.yaw : map.yaw.value_or(ref.pose.yaw);
    const double heading_error = wrap(pose.yaw - yaw);
    const double velocity_error = input.v * std::cos(heading_error) - ref.speed;
    const double position_error = distance(pose, ref.pose);
    cost += settings_.velocity_weight * square(velocity_error) +
      settings_.heading_weight * square(heading_error) +
      settings_.distance_weight * map.distance.value_or(position_error) +
      settings_.collision_weight * map.collision +
      settings_.control_change_weight[0] * square(input.v - previous.v) +
      settings_.control_change_weight[1] * square(input.omega - previous.omega);
    previous = input;
    if (states) {
      states->push_back(pose);
    }
  }
  const auto & target = refs.back().pose;
  cost += settings_.terminal_weight *
    (square(distance(pose, target)) + 0.25 * square(wrap(pose.yaw - target.yaw)));
  return std::isfinite(cost) ? cost : no_path;
}

std::optional<Solution> MppiCore::solve(
  const Pose & pose, const VOmega & previous, const Costmap & costmap, std::size_t visible_samples)
{
  if (route_.empty() || !finite(pose) || !std::isfinite(previous.v) || !std::isfinite(previous.omega)) {
    return std::nullopt;
  }
  update_progress(pose);
  const auto & goal = route_.back();
  const bool rotate_only = arc_.back() - progress_ <= 2.0 * settings_.xy_goal_tolerance &&
    distance(pose, goal) <= settings_.xy_goal_tolerance;
  if (rotate_only && std::abs(wrap(goal.yaw - pose.yaw)) <= settings_.yaw_goal_tolerance) {
    Solution result;
    result.goal_reached = true;
    result.trajectory.push_back(pose);
    warm_ = false;
    return result;
  }
  const auto refs = references(pose, rotate_only);
  const auto tracking_seed = seed(pose, refs, rotate_only);
  if (!warm_) {
    nominal_ = tracking_seed;
  }

  const std::size_t steps = refs.size();
  std::normal_distribution<double> v_noise(0.0, settings_.sigma[0]);
  std::normal_distribution<double> yaw_noise(0.0, settings_.sigma[1]);
  for (std::size_t sample = 0; sample < costs_.size(); ++sample) {
    auto * input = controls_.data() + sample * steps;
    for (std::size_t step = 0; step < steps; ++step) {
      VOmega candidate = nominal_[step];
      if (sample == 1) {
        candidate = tracking_seed[step];
      } else if (sample == 2) {
        candidate = {};
      } else if (sample > 2) {
        candidate.v += v_noise(random_);
        candidate.omega += yaw_noise(random_);
      }
      input[step] = limit(candidate, rotate_only);
    }
    costs_[sample] = rollout(pose, previous, input, refs, costmap, rotate_only);
  }

  const auto best = std::min_element(costs_.begin(), costs_.end());
  if (!std::isfinite(*best)) {
    warm_ = false;
    return std::nullopt;
  }
  std::vector<VOmega> optimum(steps);
  double total = 0.0;
  for (std::size_t sample = 0; sample < costs_.size(); ++sample) {
    const double weight = std::exp(-((costs_[sample] - *best) / settings_.lambda));
    total += weight;
    for (std::size_t step = 0; step < steps; ++step) {
      const auto & input = controls_[sample * steps + step];
      optimum[step].v += weight * input.v;
      optimum[step].omega += weight * input.omega;
    }
  }
  for (auto & input : optimum) {
    input = limit({input.v / total, input.omega / total}, rotate_only);
  }

  Solution result;
  result.cost = rollout(pose, previous, optimum.data(), refs, costmap, rotate_only, &result.trajectory);
  if (!std::isfinite(result.cost)) {
    // The mean of collision-free inputs can still produce a collision.
    const auto * feasible = controls_.data() + (best - costs_.begin()) * steps;
    std::copy_n(feasible, steps, optimum.begin());
    result.trajectory.clear();
    result.cost = rollout(pose, previous, optimum.data(), refs, costmap, rotate_only, &result.trajectory);
  }
  result.command = optimum.front();

  if (visible_samples > 0) {
    std::vector<std::size_t> order(costs_.size());
    std::iota(order.begin(), order.end(), 0);
    const auto count = std::min(visible_samples, order.size());
    std::partial_sort(order.begin(), order.begin() + count, order.end(),
      [this](std::size_t a, std::size_t b) {return costs_[a] < costs_[b];});
    for (std::size_t i = 0; i < count && std::isfinite(costs_[order[i]]); ++i) {
      auto & states = result.samples.emplace_back();
      rollout(pose, previous, controls_.data() + order[i] * steps, refs, costmap, rotate_only, &states);
    }
  }

  std::copy(optimum.begin() + 1, optimum.end(), nominal_.begin());
  nominal_.back() = optimum.back();
  warm_ = true;
  return result;
}
}  // namespace mppi_unicycle
