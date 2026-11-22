#pragma once

#include <grid_map_core/GridMap.hpp>
#include <nav_msgs/msg/path.hpp>

namespace planning
{
grid_map::GridMap make_reference_costmap(
  const nav_msgs::msg::Path & path, double resolution, double distance_limit);
}  // namespace planning
