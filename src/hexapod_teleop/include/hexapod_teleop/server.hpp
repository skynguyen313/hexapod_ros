#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hexapod_interfaces/srv/teleop_command.hpp"

namespace hexapod_teleop
{
    class TeleopServer : public rclcpp::Node
    {
        public:
            TeleopServer();
        
        private:
            void handle_command(const std::shared_ptr<hexapod_interfaces::srv::TeleopCommand::Request> request,
                                    std::shared_ptr<hexapod_interfaces::srv::TeleopCommand::Response> response);
            
            rclcpp::Service<hexapod_interfaces::srv::TeleopCommand>::SharedPtr service_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gait_pub_, speed_pub_, direction_pub_;

            std::string CMD_GAIT, CMD_SPEED, CMD_TURN, CMD_ROTATE, CMD_LIFT;
    };
}