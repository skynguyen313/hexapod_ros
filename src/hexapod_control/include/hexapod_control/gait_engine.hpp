#pragma once

#include <array>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <hexapod_interfaces/msg/leg_state_array.hpp>

namespace hexapod_control
{
    class GaitEngine
    {
        public:
            explicit GaitEngine(rclcpp::Node::SharedPtr node);
            bool hasReceivedLegs() const;
            std::vector<std::array<double,3>> generateStep(double time);

        private:
            void gaitTypeCallback(const std_msgs::msg::String::SharedPtr msg);
            void legStateCallback(const hexapod_interfaces::msg::LegStateArray::SharedPtr msg);
            double computePhase(int leg_index, double time);

            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_sub_;
            rclcpp::Subscription<hexapod_interfaces::msg::LegStateArray>::SharedPtr leg_state_sub_;

            std::string gait_type_;
            double step_height_;
            double step_length_;
            double cycle_time_;
            bool has_received_legs_;

            std::array<std::array<double, 3>, 6> neutral_positions_;    
    };
}