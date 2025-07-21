#include "hexapod_control/ik_solver.hpp"

namespace hexapod_control
{
    IKSolver::IKSolver(const rclcpp::Node::SharedPtr &node)
    {
        dims_.coxa = node->get_parameter("coxa_length").as_double();
        dims_.femur = node->get_parameter("femur_length").as_double();
        dims_.tibia = node->get_parameter("tibia_length").as_double();
    }

    std::array<double, 3> IKSolver::computeIK(double x, double y, double z)
    {
        constexpr double PI = 3.14159265358979323846;

        // Calculate coxa angle (rotation around Z axis)
        double coxa_angle = std::atan2(y, x);

        // Project leg position onto the XZ plane after subtracting coxa length
        double L1 = dims_.femur;
        double L2 = dims_.tibia;

        double horizontal = std::hypot(x, y) - dims_.coxa;
        double vertical = -z;  // downward is positive
        double dist = std::hypot(horizontal, vertical);

        if (dist > L1 + L2) {
            throw std::runtime_error("Target out of reach: IK failed");
        }

        // Use cosine law to compute angles
        double angle_b = std::acos((L1 * L1 + dist * dist - L2 * L2) / (2.0 * L1 * dist));
        double angle_c = std::acos((L1 * L1 + L2 * L2 - dist * dist) / (2.0 * L1 * L2));
        double theta = std::atan2(vertical, horizontal);

        double femur_angle = (PI / 2.0) - (theta + angle_b);
        double tibia_angle = (PI / 2.0) - angle_c;

        return {coxa_angle, femur_angle, tibia_angle};
    }

    std::vector<std::array<double, 3>> IKSolver::computeMultipleIK(const std::vector<std::array<double, 3>> &targets)
    {
        std::vector<std::array<double, 3>> results;
        results.reserve(targets.size());

        for (const auto &pos : targets)
        {
            results.emplace_back(computeIK(pos[0], pos[1], pos[2]));
        }

        return results;
    }
}