#include "mppi_4d/mppi_4d.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller::MPPI>());
    rclcpp::shutdown();
    return 0;
}
