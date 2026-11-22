#include "reference_costmap_generator/reference_costmap_generator.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planning::ReferenceCostmapGenerator>());
    rclcpp::shutdown();
    return 0;
}
