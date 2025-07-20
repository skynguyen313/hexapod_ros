#pragma once

#include <array>
#include <vector>
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hexapod_control
{
    class GaitEngine
    {
        public:
            explicit GaitEngine(rclcpp::Node::SharedPtr node);
            std::vector<std::array<double,3>> generateStep(double time);
            void gaitTypeCallback(const std_msgs::msg::String::SharedPtr msg);
            void legStateCallback(const hexapod_msgs::msg::LegStateArray::SharedPtr msg);

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_sub_;
            rclcpp::Subscription<hexapod_msgs::msg::LegStateArray>::SharedPtr leg_state_sub_;
            std::string gait_type_;
            double step_height_;
            double step_length_;
            double cycle_time_;

            std::array<std::array<double, 3>, 6> current_positions_;
            bool has_joint_state_;

            double computePhase(int leg_index, double time);
    }
}