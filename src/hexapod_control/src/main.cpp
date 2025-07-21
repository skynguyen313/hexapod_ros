#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hexapod_control/gait_engine.hpp"
#include "hexapod_control/ik_solver.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hexapod_control_node");
    
    // Declare IK Solver parameters
    node->declare_parameter("coxa_length", 0.00);
    node->declare_parameter("femur_length", 0.00);
    node->declare_parameter("tibia_length", 0.00);

    // Declare Gait Engine parameters
    node->declare_parameter("step_height", 0.00);
    node->declare_parameter("step_length", 0.00);
    node->declare_parameter("cycle_time", 0.00);
    node->declare_parameter("gait_type", "tripod");

    hexapod_control::IKSolver ik_solver(node);
    hexapod_control::GaitEngine gait_engine(node);

    auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("/hexapod/joint_states", 10);

    std::vector<std::string> joint_names = {
    "leg_1_hip_joint", "leg_1_thigh_joint", "leg_1_knee_joint",
    "leg_2_hip_joint", "leg_2_thigh_joint", "leg_2_knee_joint",
    "leg_3_hip_joint", "leg_3_thigh_joint", "leg_3_knee_joint",
    "leg_4_hip_joint", "leg_4_thigh_joint", "leg_4_knee_joint",
    "leg_5_hip_joint", "leg_5_thigh_joint", "leg_5_knee_joint",
    "leg_6_hip_joint", "leg_6_thigh_joint", "leg_6_knee_joint"
    };

    rclcpp::WallRate rate(20); // 20Hz
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        auto time_now = node->get_clock()->now().seconds();
        auto foot_positions = gait_engine.generateStep(time_now);

        if (!gait_engine.hasReceivedLegs()) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "Waiting for foot position data...");
            rate.sleep();
            continue;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node->get_clock()->now();
        msg.name = joint_names;
        
        auto joint_angles = ik_solver.computeMultipleIK(foot_positions);
        for (const auto &angles : joint_angles) {
            msg.position.insert(msg.position.end(), angles.begin(), angles.end());
        }

        joint_pub->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}