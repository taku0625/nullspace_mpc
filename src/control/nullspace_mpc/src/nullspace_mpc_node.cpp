#include "nullspace_mpc/nullspace_mpc.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller::MPC>());
    rclcpp::shutdown();
    return 0;
}
