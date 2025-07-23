#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hexapod_control
{

    class JointController
    {
        public:
            explicit JointController(rclcpp::Node::SharedPtr node);

            void publish(const std::vector<std::string>& joint_names,
                    const std::vector<std::array<double, 3>>& joint_angles);

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    };

}  // namespace hexapod_control