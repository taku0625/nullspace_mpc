#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace visualization
{
    class MapVisualizer : public rclcpp::Node
    {
        public:
            MapVisualizer();
        private:

            // publisher
            //// rviz 3D map
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_3dmap_;
            void publishRviz3DMapOfMaze(); // for map: maze
            void publishRviz3DMapOfCylinderGarden(); // for map: cylinder_garden
    };
} // namespace visualization
