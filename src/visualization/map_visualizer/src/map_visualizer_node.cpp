#include "map_visualizer/map_visualizer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<visualization::MapVisualizer>());
    rclcpp::shutdown();
    return 0;
}
