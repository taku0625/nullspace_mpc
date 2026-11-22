#include "mppi_3d/mppi_3d.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller::MPPI>());
    rclcpp::shutdown();
    return 0;
}
