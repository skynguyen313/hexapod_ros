#include "hexapod_control/joint_state_publisher.hpp"
#include <cmath>
#include <algorithm>

namespace hexapod_control
{
    JointStatePublisher::JointStatePublisher() : Node ("joint_state_publisher")
    {
        // Declare and get parameters
        this->declare_parameter("frequency", 0.0);
        this->declare_parameter("amplitude", 0.0);

        frequency_ = this->get_parameter("frequency").as_double();
        amplitude_ = this->get_parameter("amplitude").as_double();

        joint_names_ = {
            "leg_1_hip_joint", "leg_1_thigh_joint", "leg_1_knee_joint",
            "leg_2_hip_joint", "leg_2_thigh_joint", "leg_2_knee_joint",
            "leg_3_hip_joint", "leg_3_thigh_joint", "leg_3_knee_joint",
            "leg_4_hip_joint", "leg_4_thigh_joint", "leg_4_knee_joint",
            "leg_5_hip_joint", "leg_5_thigh_joint", "leg_5_knee_joint",
            "leg_6_hip_joint", "leg_6_thigh_joint", "leg_6_knee_joint"
        };

        joint_positions_.resize(joint_names_.size(), 0.0);

        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&JointStatePublisher::publish_joint_states, this));
        
        set_joint_service_ = this->create_service<hexapod_interfaces::srv::SetJointAngle>(
            "set_joint_angle", std::bind(&JointStatePublisher::handle_set_joint, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "JointStatePublisher node started with freq=%.2fHz, amp=%.2f", frequency_, amplitude_);
    }

    void JointStatePublisher::publish_joint_states()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();
        msg.name = joint_names_;

        double t = this->now().seconds();
        for (size_t i = 0; i < joint_positions_.size(); ++i)
            joint_positions_[i] = amplitude_ * std::sin(2 * M_PI * frequency_ * t + i);

        msg.position = joint_positions_;
        joint_publisher_->publish(msg);
    }

    void JointStatePublisher::handle_set_joint(std::shared_ptr<hexapod_interfaces::srv::SetJointAngle::Request> request,
                                                 std::shared_ptr<hexapod_interfaces::srv::SetJointAngle::Response> response)
    {
        auto it = std::find(joint_names_.begin(), joint_names_.end(), request->joint_name);
        if (it == joint_names_.end()) {
            response->success = false;
            response->message = "Joint name not found: " + request->joint_name;
            return;
        }

        size_t index = std::distance(joint_names_.begin(), it);
        joint_positions_[index] = request->angle;

        response->success = true;
        response->message = "Angle set successfully";
        RCLCPP_INFO(this->get_logger(), "Set joint '%s' to %.2f", request->joint_name.c_str(), request->angle);
    }
    
}   // namespace hexapod_control

