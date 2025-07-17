#include "hexapod_control/joint_state_publisher.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hexapod_control::JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}