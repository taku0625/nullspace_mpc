#include "reference_costmap_generator/reference_costmap_generator.hpp"

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <exception>

namespace planning
{
class ReferenceCostmapGenerator : public rclcpp::Node
{
public:
  ReferenceCostmapGenerator()
  : Node("reference_costmap_generator"),
    resolution_(declare_parameter("resolution", 0.05)),
    distance_limit_(declare_parameter("distance_limit", 0.60))
  {
    const auto qos = rclcpp::QoS(1).transient_local();
    publisher_ = create_publisher<grid_map_msgs::msg::GridMap>("reference_costmap", qos);
    path_ = create_subscription<nav_msgs::msg::Path>(
      "recorded_path", qos, [this](const nav_msgs::msg::Path & path) {
        const auto map = make_reference_costmap(path, resolution_, distance_limit_);
        publisher_->publish(grid_map::GridMapRosConverter::toMessage(map));
        RCLCPP_INFO(get_logger(), "Published %d x %d reference costmap in %s",
          map.getSize()(0), map.getSize()(1), map.getFrameId().c_str());
      });
  }

private:
  const double resolution_, distance_limit_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_;
};
}  // namespace planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<planning::ReferenceCostmapGenerator>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("reference_costmap_generator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
