#include "hexapod_control/controller.hpp"

namespace hexapod_control
{

    Controller::Controller(rclcpp::Node::SharedPtr node) : node_(node)
    {
        joint_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/hexapod/joint_command", 10);
    }

    void Controller::publish(const std::vector<std::string>& joint_names,
                            const std::vector<std::array<double, 3>>& joint_angles)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node_->get_clock()->now();
        msg.name = joint_names;

        for (const auto& angles : joint_angles)
            msg.position.insert(msg.position.end(), angles.begin(), angles.end());

        joint_cmd_pub_->publish(msg);
    }

}  // namespace hexapod_control
