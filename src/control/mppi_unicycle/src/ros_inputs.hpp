#pragma once

#include "mppi_unicycle/core.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/path.hpp>
#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace mppi_unicycle
{
inline Settings declare_settings(rclcpp::Node & node)
{
  Settings settings;
  settings.samples = node.declare_parameter("samples", settings.samples);
  settings.prediction_steps = node.declare_parameter("prediction_steps", settings.prediction_steps);
  settings.prediction_step = node.declare_parameter("prediction_step", settings.prediction_step);
  settings.exploration = node.declare_parameter("exploration", settings.exploration);
  settings.lambda = node.declare_parameter("lambda", settings.lambda);
  settings.alpha = node.declare_parameter("alpha", settings.alpha);
  settings.reduce_computation = node.declare_parameter(
    "reduce_computation", settings.reduce_computation);
  settings.max_linear_speed = node.declare_parameter("max_linear_speed", settings.max_linear_speed);
  settings.max_angular_speed = node.declare_parameter("max_angular_speed", settings.max_angular_speed);
  settings.reference_speed = node.declare_parameter("reference_speed", settings.reference_speed);
  settings.track_width = node.declare_parameter("track_width", settings.track_width);
  settings.wheel_speed_limit = node.declare_parameter("wheel_speed_limit", settings.wheel_speed_limit);
  settings.wheel_speed_floor = node.declare_parameter("wheel_speed_floor", settings.wheel_speed_floor);
  settings.xy_goal_tolerance = node.declare_parameter("xy_goal_tolerance", settings.xy_goal_tolerance);
  settings.yaw_goal_tolerance = node.declare_parameter("yaw_goal_tolerance", settings.yaw_goal_tolerance);
  settings.velocity_weight = node.declare_parameter("velocity_weight", settings.velocity_weight);
  settings.heading_weight = node.declare_parameter("heading_weight", settings.heading_weight);
  settings.distance_weight = node.declare_parameter("distance_weight", settings.distance_weight);
  settings.collision_weight = node.declare_parameter("collision_weight", settings.collision_weight);
  settings.terminal_weight = node.declare_parameter("terminal_weight", settings.terminal_weight);
  settings.use_sg_filter = node.declare_parameter("use_sg_filter", settings.use_sg_filter);
  settings.sg_filter_half_window_size = node.declare_parameter(
    "sg_filter_half_window_size", settings.sg_filter_half_window_size);
  settings.sg_filter_poly_order = node.declare_parameter(
    "sg_filter_poly_order", settings.sg_filter_poly_order);
  settings.actuator_dead_time = node.declare_parameter(
    "actuator_dead_time", settings.actuator_dead_time);
  settings.actuator_time_constant = node.declare_parameter(
    "actuator_time_constant", settings.actuator_time_constant);
  settings.wheel_accel_limit = node.declare_parameter("wheel_accel_limit", settings.wheel_accel_limit);
  settings.command_timeout = node.declare_parameter("command_timeout", settings.command_timeout);

  auto pair = [&node](const std::string & name, std::array<double, 2> defaults) {
      const auto value = node.declare_parameter(name, std::vector<double>{defaults[0], defaults[1]});
      if (value.size() != 2) {
        throw std::invalid_argument(name + " must contain two values in (v, omega) order");
      }
      return std::array{value[0], value[1]};
    };
  settings.sigma = pair("sigma", settings.sigma);
  settings.control_change_weight = pair("control_change_weight", settings.control_change_weight);

  const auto seed = node.declare_parameter<int64_t>("random_seed", settings.random_seed);
  if (seed < 0 || seed > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("random_seed must be an unsigned 32-bit integer");
  }
  settings.random_seed = static_cast<std::uint32_t>(seed);
  return settings;
}

inline double positive_parameter(rclcpp::Node & node, const std::string & name, double fallback)
{
  const double value = node.declare_parameter(name, fallback);
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be positive and finite");
  }
  return value;
}

inline std::optional<Pose> planar_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
  {
    return std::nullopt;
  }
  return Pose{pose.position.x, pose.position.y, tf2::getYaw(q)};
}

inline std::optional<Pose> planar_pose(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.orientation = transform.rotation;
  return planar_pose(pose);
}

inline std::optional<std::vector<Pose>> path_poses(const nav_msgs::msg::Path & path)
{
  if (path.header.frame_id.empty() || path.header.frame_id.front() == '/' || path.poses.empty()) {
    return std::nullopt;
  }
  std::vector<Pose> route;
  route.reserve(path.poses.size());
  for (const auto & point : path.poses) {
    const auto pose = planar_pose(point.pose);
    if (!pose || (!point.header.frame_id.empty() && point.header.frame_id != path.header.frame_id)) {
      return std::nullopt;
    }
    route.push_back(*pose);
  }
  return route;
}

inline double collision_at(const grid_map::GridMap & map, const grid_map::Position & position)
{
  if (map.isInside(position)) {
    const double cost = map.atPosition("collision_cost", position);
    if (std::isfinite(cost) && cost >= 0.0 && cost < 99.0) {
      return cost / 100.0;
    }
  }
  return std::numeric_limits<double>::infinity();
}
}  // namespace mppi_unicycle
