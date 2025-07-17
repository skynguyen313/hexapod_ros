#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hexapod_interfaces/srv/set_joint_angle.hpp"

namespace hexapod_control
{
    class JointStatePublisher  : public rclcpp::Node
    {
        public:
            JointStatePublisher();

        private:
            void publish_joint_states();
            void handle_set_joint(std::shared_ptr<hexapod_interfaces::srv::SetJointAngle::Request> request,
                                  std::shared_ptr<hexapod_interfaces::srv::SetJointAngle::Response> response);

            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
            rclcpp::Service<hexapod_interfaces::srv::SetJointAngle>::SharedPtr set_joint_service_;
            rclcpp::TimerBase::SharedPtr timer_;
            
            std::vector<std::string> joint_names_;
            std::vector<double> joint_positions_;

            double frequency_;
            double amplitude_;
    };
    
}   // namespace hexapod_control