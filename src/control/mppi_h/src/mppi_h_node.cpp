#include "mppi_h/mppi_h.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_mppi_h::MPPI>());
    rclcpp::shutdown();
    return 0;
}
