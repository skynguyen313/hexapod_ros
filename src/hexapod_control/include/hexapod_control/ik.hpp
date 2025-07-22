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
        double coxa;
        double femur;
        double tibia;
    };

    class IK
    {
        public:
            explicit IK(const rclcpp::Node::SharedPtr &node);
                    
            std::array<double, 3> computeIK(double x, double y, double z);
            std::vector<std::array<double, 3>> computeMultipleIK(const std::vector<std::array<double, 3>> &targets);

        private:
            LegDimensions dims_;
    };

}   // namespace hexapod_control