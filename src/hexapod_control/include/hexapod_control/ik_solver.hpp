#pragma once

#include <array>
#include <string>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace hexapod_control
{
    struct LegDimensions
    {
        double coxa;   // length from body center to first joint
        double femur;  // upper leg
        double tibia;  // lower leg
    };

    class IKSolver
    {
        public:
            explicit IKSolver(const rclcpp::Node::SharedPtr &node);

            // Compute IK for a given leg tip position (in meters)
            std::array<double, 3> computeIK(double x, double y, double z);
            
            // Compute IK for multiple legs
            std::vector<std::array<double, 3>> computeMultipleIK(const std::vector<std::array<double, 3>> &targets);

        private:
            LegDimensions dims_;

    };  // namespace hexapod_control

}