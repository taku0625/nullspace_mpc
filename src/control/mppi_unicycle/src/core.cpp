#include "mppi_unicycle/core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

namespace mppi_unicycle
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double no_path = std::numeric_limits<double>::infinity();
// Limits of the prediction of the rollout start, in seconds: the replay of the wheel speeds
// before the pose time, the age of a pose, and the step of the pose between them.
constexpr double max_settling_time = 10.0;
constexpr double max_pose_age = 2.0;
constexpr double pose_step = 0.02;

double square(double value)
{
  return value * value;
}

bool finite(const Pose & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

std::uint64_t mix(std::uint64_t value)
{
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

// SplitMix64. Its state is one integer, so every sample can run its own stream. The noise is
// then drawn in parallel and does not depend on the number of threads.
struct SampleRandom
{
  using result_type = std::uint64_t;
  static constexpr result_type min() {return 0;}
  static constexpr result_type max() {return std::numeric_limits<result_type>::max();}
  result_type operator()() {return mix(state += 0x9e3779b97f4a7c15ULL);}
  result_type state;
};
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

namespace
{
// Moves the pose with the heading it has halfway through the turn. The mean velocity of a wheel
// response over the step places the pose closer than the heading at the start of the step does.
Pose advance_midway(const Pose & pose, const VOmega & mean, double dt)
{
  const double turn = dt * mean.omega;
  return {
    pose.x + dt * mean.v * std::cos(pose.yaw + 0.5 * turn),
    pose.y + dt * mean.v * std::sin(pose.yaw + 0.5 * turn),
    wrap(pose.yaw + turn),
  };
}

// Moves one wheel toward a constant target for dt and returns its mean speed over that time.
// The reference ramps to the target at the acceleration limit and rests there. The speed
// follows the reference with the time constant. Both phases have a closed solution, so the
// result does not depend on how a time span is divided into steps.
double drive_wheel(
  double & reference, double & speed, double target, double dt, double time_constant,
  double accel_limit)
{
  if (!(dt > 0.0)) {
    return speed;
  }
  const double gap = target - reference;
  const double ramp = accel_limit > 0.0 ? std::min(dt, std::abs(gap) / accel_limit) : 0.0;
  double travelled = 0.0;
  if (ramp > 0.0) {
    const double slope = std::copysign(accel_limit, gap);
    const double reached = reference + slope * ramp;
    if (time_constant > 0.0) {
      // Behind a ramping reference the speed settles at the reference minus slope * tau.
      const double decay = std::exp(-ramp / time_constant);
      const double offset = speed - reference + slope * time_constant;
      travelled += (reference - slope * time_constant) * ramp + 0.5 * slope * ramp * ramp +
        offset * time_constant * (1.0 - decay);
      speed = reached - slope * time_constant + offset * decay;
    } else {
      travelled += (reference + 0.5 * slope * ramp) * ramp;
      speed = reached;
    }
    reference = reached;
  }
  const double rest = dt - ramp;
  if (rest > 0.0) {
    reference = target;
    if (time_constant > 0.0) {
      const double decay = std::exp(-rest / time_constant);
      const double offset = speed - target;
      travelled += target * rest + offset * time_constant * (1.0 - decay);
      speed = target + offset * decay;
    } else {
      travelled += target * rest;
      speed = target;
    }
  }
  return travelled / dt;
}
}  // namespace

std::vector<double> savitzky_golay_coefficients(int half_window, int order)
{
  if (half_window < 1 || order < 0 || order > 2 * half_window) {
    throw std::invalid_argument(
            "The Savitzky-Golay filter needs a half window of at least one sample and a "
            "polynomial order of at most twice the half window");
  }
  const int terms = order + 1;
  const int window = 2 * half_window + 1;
  // Sample positions scaled to [-1, 1] keep the normal equations well conditioned.
  std::vector<double> positions(window);
  for (int i = 0; i < window; ++i) {
    positions[i] = static_cast<double>(i - half_window) / half_window;
  }

  // Solve (X^T X) z = e_0 for the Vandermonde matrix X of the positions. The weights are X z.
  std::vector<double> matrix(terms * terms, 0.0);
  std::vector<double> z(terms, 0.0);
  z[0] = 1.0;
  std::vector<double> powers(2 * terms - 1);
  for (const double position : positions) {
    powers[0] = 1.0;
    for (std::size_t i = 1; i < powers.size(); ++i) {
      powers[i] = powers[i - 1] * position;
    }
    for (int row = 0; row < terms; ++row) {
      for (int column = 0; column < terms; ++column) {
        matrix[row * terms + column] += powers[row + column];
      }
    }
  }
  for (int column = 0; column < terms; ++column) {
    int pivot = column;
    for (int row = column + 1; row < terms; ++row) {
      if (std::abs(matrix[row * terms + column]) > std::abs(matrix[pivot * terms + column])) {
        pivot = row;
      }
    }
    std::swap_ranges(
      matrix.begin() + pivot * terms, matrix.begin() + (pivot + 1) * terms,
      matrix.begin() + column * terms);
    std::swap(z[pivot], z[column]);
    for (int row = column + 1; row < terms; ++row) {
      const double factor = matrix[row * terms + column] / matrix[column * terms + column];
      for (int i = column; i < terms; ++i) {
        matrix[row * terms + i] -= factor * matrix[column * terms + i];
      }
      z[row] -= factor * z[column];
    }
  }
  for (int row = terms - 1; row >= 0; --row) {
    for (int i = row + 1; i < terms; ++i) {
      z[row] -= matrix[row * terms + i] * z[i];
    }
    z[row] /= matrix[row * terms + row];
  }

  std::vector<double> weights(window);
  for (int i = 0; i < window; ++i) {
    for (int term = terms - 1; term >= 0; --term) {
      weights[i] = weights[i] * positions[i] + z[term];
    }
  }
  return weights;
}

MppiCore::MppiCore(Settings settings)
: settings_(settings)
{
  const std::array positive{
    settings.prediction_step, settings.sigma[0], settings.sigma[1], settings.lambda,
    settings.max_linear_speed, settings.max_angular_speed, settings.reference_speed,
    settings.track_width, settings.xy_goal_tolerance, settings.yaw_goal_tolerance,
  };
  const std::array nonnegative{
    settings.velocity_weight, settings.heading_weight, settings.distance_weight,
    settings.collision_weight, settings.terminal_weight, settings.control_change_weight[0],
    settings.control_change_weight[1], settings.wheel_speed_limit, settings.wheel_speed_floor,
    settings.actuator_dead_time, settings.actuator_time_constant, settings.wheel_accel_limit,
  };
  if (settings.samples < 3 || settings.prediction_steps < 2 ||
    !std::all_of(positive.begin(), positive.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;}) ||
    !std::all_of(nonnegative.begin(), nonnegative.end(),
      [](double value) {return std::isfinite(value) && value >= 0.0;}) ||
    !(settings.exploration >= 0.0 && settings.exploration <= 1.0) ||
    !(settings.alpha >= 0.0 && settings.alpha <= 1.0))
  {
    throw std::invalid_argument("Invalid MPPI horizon, noise, limits or cost weights");
  }
  if (!std::isfinite(settings.command_timeout) || settings.command_timeout <= 0.0) {
    throw std::invalid_argument("The command timeout must be positive and finite");
  }
  // With a floor the robot drives with both wheels at it or faster and turns in place with
  // both wheels at it, so the speed limits have to leave room for both.
  const double floor = settings.wheel_speed_floor;
  if (floor > settings.max_linear_speed ||
    (settings.wheel_speed_limit > 0.0 && floor > settings.wheel_speed_limit) ||
    floor > 0.5 * settings.track_width * settings.max_angular_speed)
  {
    throw std::invalid_argument(
            "The wheel speed floor exceeds the linear, angular or wheel speed limit");
  }
  wheel_min_speed_ = floor;
  if (settings.use_sg_filter) {
    if (settings.sg_filter_half_window_size >= settings.prediction_steps) {
      throw std::invalid_argument(
              "The Savitzky-Golay half window must be shorter than the prediction horizon");
    }
    filter_coefficients_ = savitzky_golay_coefficients(
      settings.sg_filter_half_window_size, settings.sg_filter_poly_order);
    filter_history_.resize(settings.sg_filter_half_window_size);
  }
  const auto size = static_cast<std::size_t>(settings.samples) * settings.prediction_steps;
  noise_.resize(size);
  controls_.resize(size);
  states_.resize(size);
  costs_.resize(settings.samples);
  nominal_.resize(settings.prediction_steps);
  sample_noise();
}

bool MppiCore::set_wheel_speed_limits(double minimum, double maximum)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum < 0.0 || maximum <= 0.0 ||
    minimum > std::min(maximum, settings_.max_linear_speed))
  {
    return false;
  }
  if (minimum != wheel_min_speed_ || maximum != settings_.wheel_speed_limit) {
    wheel_min_speed_ = minimum;
    settings_.wheel_speed_limit = maximum;
    maneuver_ = Maneuver::drive;
    reset();
  }
  return true;
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
  maneuver_ = Maneuver::drive;
  reset();
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

double MppiCore::project(const Pose & pose, double progress) const
{
  if (progress >= arc_.back()) {
    return progress;
  }
  // Bound projection to nearby segments to preserve ordering at path crossings.
  const double end = std::min(arc_.back(), progress + std::max(0.4, settings_.max_linear_speed * 0.3));
  auto segment = std::upper_bound(arc_.begin(), arc_.end(), progress) - arc_.begin();
  double closest_arc = progress;
  double closest_distance = distance(pose, pose_at(progress));
  for (; segment < static_cast<decltype(segment)>(arc_.size()) && arc_[segment - 1] < end; ++segment) {
    const auto & a = route_[segment - 1];
    const auto & b = route_[segment];
    const double span = arc_[segment] - arc_[segment - 1];
    const double along = ((pose.x - a.x) * (b.x - a.x) + (pose.y - a.y) * (b.y - a.y)) / span;
    const double projected = std::clamp(
      arc_[segment - 1] + along, std::max(arc_[segment - 1], progress),
      std::min(arc_[segment], end));
    const double error = distance(pose, pose_at(projected));
    if (error < closest_distance) {
      closest_distance = error;
      closest_arc = projected;
    }
  }
  return closest_arc;
}

void MppiCore::update_progress(const Pose & pose)
{
  progress_ = project(pose, progress_);
}

VOmega MppiCore::limit(VOmega input, bool rotate_only) const
{
  input.v = rotate_only ? 0.0 : std::clamp(input.v, 0.0, settings_.max_linear_speed);
  input.omega = std::clamp(input.omega, -settings_.max_angular_speed, settings_.max_angular_speed);
  const double half_track = 0.5 * settings_.track_width;
  if (wheel_min_speed_ > 0.0) {
    // Forward rollouts keep both wheels at the floor or faster. Sharper turns are made in place.
    const double fastest = settings_.wheel_speed_limit > 0.0 ?
      settings_.wheel_speed_limit : no_path;
    input.v = std::clamp(input.v, wheel_min_speed_, std::min(settings_.max_linear_speed, fastest));
    const double max_turn = std::min(input.v - wheel_min_speed_, fastest - input.v) / half_track;
    input.omega = std::clamp(input.omega, -max_turn, max_turn);
    return input;
  }
  const double fastest_wheel = input.v + half_track * std::abs(input.omega);
  if (settings_.wheel_speed_limit > 0.0 && fastest_wheel > settings_.wheel_speed_limit) {
    const double scale = settings_.wheel_speed_limit / fastest_wheel;
    input.v *= scale;
    input.omega *= scale;
  }
  return input;
}

MppiCore::Tracking MppiCore::track(bool rotate_only, double progress) const
{
  const double horizon = settings_.prediction_steps * settings_.prediction_step;
  const double reach = std::min(settings_.reference_speed, settings_.max_linear_speed) * horizon;
  Tracking tracking{
    rotate_only, rotate_only || arc_.back() - progress <= reach, pose_at(progress + reach),
    2.0 * settings_.xy_goal_tolerance, {}};
  const double end = progress + settings_.max_linear_speed * horizon + tracking.lookahead;
  auto segment = std::min<std::size_t>(
    std::upper_bound(arc_.begin(), arc_.end(), progress) - arc_.begin(), arc_.size() - 1);
  for (; segment > 0 && segment < arc_.size() && arc_[segment - 1] <= end; ++segment) {
    const auto & a = route_[segment - 1];
    const auto & b = route_[segment];
    const double length = arc_[segment] - arc_[segment - 1];
    tracking.window.push_back({a.x, a.y, b.x - a.x, b.y - a.y, length});
  }
  return tracking;
}

void MppiCore::sample_noise()
{
  const int steps = settings_.prediction_steps;
  const std::uint64_t draw = mix(mix(settings_.random_seed) + noise_draws_++);
  #pragma omp parallel for schedule(static)
  for (int sample = 0; sample < settings_.samples; ++sample) {
    SampleRandom random{mix(draw + static_cast<std::uint64_t>(sample))};
    std::normal_distribution<double> v_noise(0.0, settings_.sigma[0]);
    std::normal_distribution<double> omega_noise(0.0, settings_.sigma[1]);
    auto * noise = noise_.data() + static_cast<std::size_t>(sample) * steps;
    for (int step = 0; step < steps; ++step) {
      noise[step].v = v_noise(random);
      noise[step].omega = omega_noise(random);
    }
  }
}

bool MppiCore::wheels_modelled() const
{
  return settings_.actuator_time_constant > 0.0 || settings_.wheel_accel_limit > 0.0;
}

VOmega MppiCore::drive(Wheels & wheels, const VOmega & command, double dt) const
{
  const double half_track = 0.5 * settings_.track_width;
  const std::array<double, 2> target{
    command.v - half_track * command.omega, command.v + half_track * command.omega};
  std::array<double, 2> mean;
  for (std::size_t i = 0; i < target.size(); ++i) {
    mean[i] = drive_wheel(
      wheels.reference[i], wheels.speed[i], target[i], dt, settings_.actuator_time_constant,
      settings_.wheel_accel_limit);
  }
  return {0.5 * (mean[0] + mean[1]), (mean[1] - mean[0]) / settings_.track_width};
}

void MppiCore::command_sent(const VOmega & command, double time)
{
  if (!std::isfinite(time) || !std::isfinite(command.v) || !std::isfinite(command.omega)) {
    return;
  }
  if (!sent_.empty() && time < sent_.back().time) {
    sent_.clear();  // The clock was set back.
  }
  sent_.push_back({time, command});
  // A replay looks back by the settling time of the wheels and the age of the pose. One older
  // command stays, because the wheels were following it at the start of the replay.
  const double oldest = time - (max_settling_time + max_pose_age + 1.0) -
    settings_.actuator_dead_time;
  while (sent_.size() > 1 && sent_[1].time < oldest) {
    sent_.pop_front();
  }
}

std::optional<MppiCore::Start> MppiCore::predict(const Pose & pose, const Timing & timing) const
{
  if (!std::isfinite(timing.pose_time) || !std::isfinite(timing.command_time)) {
    return std::nullopt;
  }
  const double dead_time = settings_.actuator_dead_time;
  const bool modelled = wheels_modelled();
  // The command of this solve starts to act on the wheels at the end of the prediction.
  const double end = timing.command_time + dead_time;
  const double begin = std::clamp(timing.pose_time, end - max_pose_age, end);
  if (!modelled && begin == end) {
    return std::nullopt;
  }

  // The wheel speeds at the pose time come from the commands before it. The replay starts early
  // enough for the wheels to have forgotten the speeds they are given there.
  double settling = 0.0;
  if (modelled) {
    settling = 5.0 * settings_.actuator_time_constant;
    if (settings_.wheel_accel_limit > 0.0) {
      const double fastest = settings_.wheel_speed_limit > 0.0 ? settings_.wheel_speed_limit :
        settings_.max_linear_speed + 0.5 * settings_.track_width * settings_.max_angular_speed;
      settling += 2.0 * fastest / settings_.wheel_accel_limit;
    }
    settling = std::min(settling, max_settling_time);
  }
  double time = begin - settling;
  const auto arrival = [&](std::size_t index) {return sent_[index].time + dead_time;};
  // The command at this index is the next one to reach the wheels. The one before it acts on
  // them until it times out.
  std::size_t next = 0;
  while (next < sent_.size() && arrival(next) <= time) {
    ++next;
  }
  Start start{pose, {}, modelled};
  if (modelled && next > 0 && time < arrival(next - 1) + settings_.command_timeout) {
    // The wheels have settled on the command that acts on them at the start of the replay.
    const auto & command = sent_[next - 1].command;
    const double half_track = 0.5 * settings_.track_width;
    start.wheels.reference = {
      command.v - half_track * command.omega, command.v + half_track * command.omega};
    start.wheels.speed = start.wheels.reference;
  }
  while (time < end) {
    VOmega command;
    double until = end;
    if (next > 0) {
      const double expiry = arrival(next - 1) + settings_.command_timeout;
      if (time < expiry) {
        command = sent_[next - 1].command;
        until = std::min(until, expiry);
      }
    }
    if (next < sent_.size()) {
      until = std::min(until, arrival(next));
    }
    // The pose moves in short steps from the time it was measured.
    until = std::min(until, time < begin ? begin : time + pose_step);
    const double dt = until - time;
    const VOmega motion = modelled ? drive(start.wheels, command, dt) : command;
    if (time >= begin) {
      start.pose = advance_midway(start.pose, motion, dt);
    }
    time = until;
    while (next < sent_.size() && arrival(next) <= time) {
      ++next;
    }
  }
  return start;
}

double MppiCore::stage_cost(
  const Pose & state, double forward_speed, const VOmega & input, const VOmega & previous,
  const Tracking & tracking, const Costmap & costmap) const
{
  const auto map = costmap ? costmap(state) : MapCost{};
  const auto & goal = route_.back();
  const double goal_distance = distance(state, goal);

  // Nearest point of the route section ahead of the robot.
  double route_distance = goal_distance;
  Pose ahead = goal;
  if (!tracking.window.empty()) {
    std::size_t nearest = 0;
    double along = 0.0;
    double closest = no_path;
    for (std::size_t i = 0; i < tracking.window.size(); ++i) {
      const auto & segment = tracking.window[i];
      const double x = state.x - segment.x, y = state.y - segment.y;
      const double ratio = std::clamp(
        (x * segment.dx + y * segment.dy) / square(segment.length), 0.0, 1.0);
      const double error = square(x - ratio * segment.dx) + square(y - ratio * segment.dy);
      if (error < closest) {
        closest = error;
        nearest = i;
        along = ratio * segment.length;
      }
    }
    route_distance = std::sqrt(closest);
    // Walk the lookahead distance along the route from there.
    along += tracking.lookahead;
    while (along > tracking.window[nearest].length && nearest + 1 < tracking.window.size()) {
      along -= tracking.window[nearest++].length;
    }
    const auto & segment = tracking.window[nearest];
    const double ratio = std::min(along / segment.length, 1.0);
    ahead = {segment.x + ratio * segment.dx, segment.y + ratio * segment.dy,
      std::atan2(segment.dy, segment.dx)};
  }

  double cost = 0.0;
  if (tracking.goal_in_reach && goal_distance <= settings_.xy_goal_tolerance) {
    // A unicycle cannot correct its heading after it stops, so it turns to the goal yaw here.
    cost += settings_.heading_weight * square(wrap(state.yaw - goal.yaw));
  } else {
    // The reference direction points at the route ahead. It follows the order of the route where
    // the recording passes the same place twice, and it turns gradually before a corner
    // recorded as a turn in place, where the direction of the nearest segment would jump.
    const double yaw = distance(state, ahead) > 1e-6 ?
      std::atan2(ahead.y - state.y, ahead.x - state.x) : ahead.yaw;
    const double heading_error = wrap(state.yaw - yaw);
    const double speed = std::min(settings_.reference_speed, settings_.max_linear_speed);
    cost += settings_.velocity_weight * square(forward_speed * std::cos(heading_error) - speed) +
      settings_.heading_weight * square(heading_error);
  }
  return cost + settings_.collision_weight * map.collision +
         settings_.distance_weight * map.distance.value_or(route_distance) +
         settings_.control_change_weight[0] * square(input.v - previous.v) +
         settings_.control_change_weight[1] * square(input.omega - previous.omega);
}

double MppiCore::rollout(
  const Start & start, const VOmega * controls, Pose * states, const Tracking & tracking,
  const Costmap & costmap) const
{
  const double input_weight = settings_.lambda * (1.0 - settings_.alpha);
  Pose state = start.pose;
  Wheels wheels = start.wheels;
  VOmega previous = last_command_;
  double cost = 0.0;
  for (int step = 0; step < settings_.prediction_steps; ++step) {
    const auto & input = controls[step];
    double forward_speed = input.v;
    if (start.wheels_modelled) {
      // The robot moves at the speeds the wheels reach, which trail the inputs.
      const auto motion = drive(wheels, input, settings_.prediction_step);
      state = advance_midway(state, motion, settings_.prediction_step);
      forward_speed = motion.v;
    } else {
      state = advance(state, input, settings_.prediction_step);
    }
    if (states) {
      states[step] = state;
    }
    cost += stage_cost(state, forward_speed, input, previous, tracking, costmap) +
      input_weight * (nominal_[step].v * input.v / settings_.sigma[0] +
      nominal_[step].omega * input.omega / settings_.sigma[1]);
    previous = input;
  }
  const double error = distance(state, tracking.target);
  if (error > settings_.xy_goal_tolerance) {
    cost += settings_.terminal_weight * square(error);
  }
  // A lethal or unknown cell makes the cost infinite or, with a zero weight, not a number.
  return std::isfinite(cost) ? cost : no_path;
}

VOmega MppiCore::smooth(const std::vector<VOmega> & controls)
{
  // The window holds the past commands, the new command and the inputs planned after it.
  const std::size_t half = filter_history_.size();
  VOmega filtered;
  for (std::size_t i = 0; i < filter_coefficients_.size(); ++i) {
    const auto & input = i < half ? filter_history_[i] : controls[i - half];
    filtered.v += filter_coefficients_[i] * input.v;
    filtered.omega += filter_coefficients_[i] * input.omega;
  }
  std::rotate(filter_history_.begin(), filter_history_.begin() + 1, filter_history_.end());
  filter_history_.back() = filtered;
  return filtered;
}

void MppiCore::reset()
{
  // With a wheel speed floor the plan starts at the slowest forward speed. Around zero input
  // every sample would be raised to that same speed, and their costs would not tell them apart.
  std::fill(nominal_.begin(), nominal_.end(), limit({}, false));
  std::fill(filter_history_.begin(), filter_history_.end(), VOmega{});
  last_command_ = {};
}

std::optional<Solution> MppiCore::solve(
  const Pose & pose, const VOmega & measured, const Costmap & costmap)
{
  return optimize(pose, measured, costmap, nullptr);
}

std::optional<Solution> MppiCore::solve(
  const Pose & pose, const VOmega & measured, const Costmap & costmap, const Timing & timing)
{
  return optimize(pose, measured, costmap, &timing);
}

std::optional<Solution> MppiCore::optimize(
  const Pose & pose, const VOmega & measured, const Costmap & costmap, const Timing * timing)
{
  const auto started = std::chrono::steady_clock::now();
  optimal_.clear();
  if (route_.empty() || !finite(pose) || !std::isfinite(measured.v) ||
    !std::isfinite(measured.omega))
  {
    return std::nullopt;
  }
  update_progress(pose);
  const auto & goal = route_.back();
  const bool rotate_only = arc_.back() - progress_ <= 2.0 * settings_.xy_goal_tolerance &&
    distance(pose, goal) <= settings_.xy_goal_tolerance;
  if (rotate_only && std::abs(wrap(goal.yaw - pose.yaw)) <= settings_.yaw_goal_tolerance) {
    reset();
    return Solution{{}, true};
  }
  if (wheel_min_speed_ > 0.0) {
    const auto lookahead = pose_at(progress_ + 2.0 * settings_.xy_goal_tolerance);
    const double target = rotate_only ? goal.yaw :
      std::atan2(lookahead.y - pose.y, lookahead.x - pose.x);
    const double error = wrap(target - pose.yaw);
    const double half_track = 0.5 * settings_.track_width;
    if (maneuver_ == Maneuver::drive && (rotate_only || std::abs(error) > pi / 4.0)) {
      maneuver_ = Maneuver::brake;
    } else if (maneuver_ == Maneuver::turn) {
      // A sign change near half a turn is the angle wrapping, not the target being passed.
      const bool passed = error * turn_direction_ < 0.0 && std::abs(error) < pi / 2.0;
      if (passed || (!rotate_only && std::abs(error) <= pi / 8.0)) {
        maneuver_ = Maneuver::brake;
      }
    }
    if (maneuver_ == Maneuver::brake) {
      // Sensorless ESCs start a wheel at the floor speed and reverse it only from rest. Both
      // wheels stop between driving and turning so that they start the next maneuver together.
      reset();
      const double creep = 0.5 * wheel_min_speed_;
      if (std::abs(measured.v) > creep || half_track * std::abs(measured.omega) > creep) {
        return Solution{};
      }
      if (rotate_only || std::abs(error) > pi / 8.0) {
        maneuver_ = Maneuver::turn;
        turn_direction_ = std::copysign(1.0, error);
      } else {
        maneuver_ = Maneuver::drive;
      }
    }
    if (maneuver_ == Maneuver::turn) {
      reset();
      const double min_turn = wheel_min_speed_ / half_track;
      const double max_turn = settings_.wheel_speed_limit > 0.0 ?
        std::min(settings_.max_angular_speed, settings_.wheel_speed_limit / half_track) :
        settings_.max_angular_speed;
      if (max_turn < min_turn || (costmap && !std::isfinite(costmap(pose).collision))) {
        return std::nullopt;
      }
      // The direction is kept for the whole turn so the ESCs can complete acquisition.
      return Solution{{0.0, turn_direction_ *
        std::clamp(3.0 * std::abs(error), min_turn, max_turn)}, false};
    }
  }

  // The measured pose decided the progress and the maneuver above. The rollouts start where the
  // robot is predicted to be when this command reaches the wheels, and follow the route from
  // the position of that pose along it.
  Start start{pose, {}, false};
  double progress = progress_;
  if (timing) {
    if (const auto predicted = predict(pose, *timing)) {
      start = *predicted;
      progress = project(start.pose, progress_);
    }
  }
  const auto tracking = track(rotate_only, progress);
  const int samples = settings_.samples;
  const int steps = settings_.prediction_steps;
  if (!settings_.reduce_computation) {
    sample_noise();
  }

  // Sample around the previous solution, and around zero input for the exploring share.
  const int exploiting = static_cast<int>((1.0 - settings_.exploration) * samples);
  #pragma omp parallel for schedule(static)
  for (int sample = 0; sample < samples; ++sample) {
    const auto row = static_cast<std::size_t>(sample) * steps;
    for (int step = 0; step < steps; ++step) {
      VOmega input = noise_[row + step];
      if (sample < exploiting) {
        input.v += nominal_[step].v;
        input.omega += nominal_[step].omega;
      }
      controls_[row + step] = limit(input, rotate_only);
    }
    costs_[sample] = rollout(
      start, controls_.data() + row, states_.data() + row, tracking, costmap);
  }

  const auto best = std::min_element(costs_.begin(), costs_.end());
  if (!std::isfinite(*best)) {
    reset();
    return std::nullopt;
  }

  // Add the noise of each sample to the previous solution, weighted by its cost.
  double total = 0.0;
  for (const double cost : costs_) {
    total += std::exp(-(cost - *best) / settings_.lambda);
  }
  auto optimum = nominal_;
  for (int sample = 0; sample < samples; ++sample) {
    const double weight = std::exp(-(costs_[sample] - *best) / settings_.lambda) / total;
    const auto * noise = noise_.data() + static_cast<std::size_t>(sample) * steps;
    for (int step = 0; step < steps; ++step) {
      optimum[step].v += weight * noise[step].v;
      optimum[step].omega += weight * noise[step].omega;
    }
  }
  if (settings_.use_sg_filter) {
    optimum.front() = smooth(optimum);
  }
  for (auto & input : optimum) {
    input = limit(input, rotate_only);
  }

  optimal_.resize(steps + 1);
  optimal_.front() = start.pose;
  if (!std::isfinite(rollout(start, optimum.data(), optimal_.data() + 1, tracking, costmap))) {
    // The weighted mean of collision-free inputs can still produce a collision.
    const auto row = static_cast<std::size_t>(best - costs_.begin()) * steps;
    std::copy_n(controls_.begin() + row, steps, optimum.begin());
    std::copy_n(states_.begin() + row, steps, optimal_.begin() + 1);
  }

  last_command_ = optimum.front();
  nominal_ = std::move(optimum);
  calculation_time_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return Solution{last_command_, false};
}

const std::vector<Pose> & MppiCore::optimal_trajectory() const
{
  return optimal_;
}

std::vector<std::vector<Pose>> MppiCore::elite_trajectories(std::size_t count) const
{
  std::vector<std::vector<Pose>> trajectories;
  if (optimal_.empty()) {
    return trajectories;
  }
  std::vector<std::size_t> rank(costs_.size());
  std::iota(rank.begin(), rank.end(), 0);
  count = std::min(count, rank.size());
  std::partial_sort(rank.begin(), rank.begin() + count, rank.end(),
    [this](std::size_t a, std::size_t b) {return costs_[a] < costs_[b];});
  const std::size_t steps = settings_.prediction_steps;
  for (std::size_t i = 0; i < count && std::isfinite(costs_[rank[i]]); ++i) {
    const auto row = states_.begin() + rank[i] * steps;
    trajectories.emplace_back(row, row + steps);
  }
  return trajectories;
}

double MppiCore::calculation_time_ms() const
{
  return calculation_time_ms_;
}
}  // namespace mppi_unicycle
