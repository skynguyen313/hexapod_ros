#include "hexapod_control/gait.hpp"
#include <cmath>

namespace hexapod_control
{

    Gait::Gait(rclcpp::Node::SharedPtr node) : node_(node), has_received_legs_(false)
    {
        step_height_ = node_->get_parameter("step_height").as_double();
        step_length_ = node_->get_parameter("step_length").as_double();
        cycle_time_  = node_->get_parameter("cycle_time").as_double();
        gait_type_   = node_->get_parameter("gait_type").as_string();

        // Subscriptions
        gait_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/hexapod/gait_type", 10,
            std::bind(&Gait::gaitTypeCallback, this, std::placeholders::_1)
        );
        
        leg_state_sub_ = node_->create_subscription<hexapod_interfaces::msg::LegStateArray>(
            "/hexapod/leg_states", 10,
            std::bind(&Gait::legStateCallback, this, std::placeholders::_1)
        );

        neutral_positions_.fill({NAN, NAN, NAN});
    }

    void Gait::gaitTypeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        gait_type_ = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Gait type updated to: %s", gait_type_.c_str());
    }

    void Gait::legStateCallback(const hexapod_interfaces::msg::LegStateArray::SharedPtr msg)
    {
        if (msg->legs.size() != 6) {
            RCLCPP_WARN(node_->get_logger(), "Received leg state array with wrong size!");
            return;
        }

        for (size_t i = 0; i < 6; ++i) {
            neutral_positions_[i][0] = msg->legs[i].position.x;
            neutral_positions_[i][1] = msg->legs[i].position.y;
            neutral_positions_[i][2] = msg->legs[i].position.z;
        }

        has_received_legs_ = true;
    }

    double Gait::computePhase(int leg_index, double time)
    {
        if (gait_type_ == "tripod") {
            return std::fmod(time + (leg_index % 2 == 0 ? 0.0 : cycle_time_ / 2), cycle_time_) / cycle_time_;
        }
        // Add more gait types here
        return std::fmod(time, cycle_time_) / cycle_time_;
    }

    bool Gait::hasReceivedLegs() const { return has_received_legs_; }

    std::vector<std::array<double, 3>> Gait::generateStep(double time)
    {
        std::vector<std::array<double, 3>> steps(6);

        if (!has_received_legs_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "Haven't received leg state data yet. Skipping step generation.");
            return steps;
        }

        for (int i = 0; i < 6; ++i)
        {
            double phase = computePhase(i, time);
            double x_offset = step_length_ * std::cos(2 * M_PI * phase - M_PI);
            double z_offset = (phase < 0.5) ? step_height_ * std::sin(M_PI * phase * 2) : 0.0;

            steps[i][0] = neutral_positions_[i][0] + x_offset;
            steps[i][1] = neutral_positions_[i][1];
            steps[i][2] = neutral_positions_[i][2] + z_offset;
        }

        return steps;
    }
    
} // namespace hexapod_control
