#include "rclcpp/rclcpp.hpp"
#include "hexapod_control/gait.hpp"
#include "hexapod_control/ik.hpp"
#include "hexapod_control/controller.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hexapod_control");

    // Params
    node->declare_parameter("coxa_length",  0.00);
    node->declare_parameter("femur_length", 0.00);
    node->declare_parameter("tibia_length", 0.00);
    node->declare_parameter("step_height",  0.00);
    node->declare_parameter("step_length",  0.00);
    node->declare_parameter("cycle_time",   0.00);
    node->declare_parameter("gait_type",      "");

    hexapod_control::Gait gait(node);
    hexapod_control::IK ik(node);
    hexapod_control::Controller controller(node);

    const std::vector<std::string> joint_names = {
        "leg_1_hip_joint", "leg_1_thigh_joint", "leg_1_knee_joint",
        "leg_2_hip_joint", "leg_2_thigh_joint", "leg_2_knee_joint",
        "leg_3_hip_joint", "leg_3_thigh_joint", "leg_3_knee_joint",
        "leg_4_hip_joint", "leg_4_thigh_joint", "leg_4_knee_joint",
        "leg_5_hip_joint", "leg_5_thigh_joint", "leg_5_knee_joint",
        "leg_6_hip_joint", "leg_6_thigh_joint", "leg_6_knee_joint",
    };

    rclcpp::WallRate rate(20); // 20Hz

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (!gait.hasReceivedLegs()) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "Waiting for leg state data...");
            rate.sleep();
            continue;
        }

        double time_now = node->get_clock()->now().seconds();
        auto foot_positions = gait.generateStep(time_now);
        auto joint_angles = ik.computeMultipleIK(foot_positions);

        controller.publish(joint_names, joint_angles);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
