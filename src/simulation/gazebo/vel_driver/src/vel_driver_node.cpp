#include "vel_driver/vel_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gazebo::VelDriver>());
    rclcpp::shutdown();
    return 0;
}
