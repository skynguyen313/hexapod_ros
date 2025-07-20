#include "hexapod_control/gait_engine.hpp"
#include <cmath>

namespace hexapod_control
{
    GaitEngine::GaitEngine(rclcpp::Node::SharedPtr node) : node_(node)
    {
        // Params
        node_->declare_parameter("step_height", 0.00);
        node_->declare_parameter("step_length", 0.00);
        node_->declare_parameter("cycle_time", 0.00);
        node_->declare_parameter("gait_type", "");

        node_->get_parameter("step_height", step_height_);
        node_->get_parameter("step_length", step_length_);
        node_->get_parameter("cycle_time", cycle_time_);
        node_->get_parameter("gait_type", gait_type_);

        // Subscriptions
        gait_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/hexapod/gait_type", 10,
            std::bind(&GaitEngine::gaitTypeCallback, this, std::placeholders::_1)
        );
        
        leg_state_sub_ = node_->create_subscription<hexapod_msgs::msg::LegStateArray>(
            "/hexapod/leg_states", 10,
            std::bind(&GaitEngine::legStateCallback, this, std::placeholders::_1)
        );

        current_positions_.fill({NAN, NAN, NAN});
        has_received_position_ = false;
    }

    std::vector<std::array<double, 3>> GaitEngine::generateStep(double time)
    {
        std::vector<std::array<double, 3>> foot_positions(6);

        if (!has_joint_state_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "Haven't received leg state data yet. Skipping step generation.");
            return foot_positions;
        }

        for (int i = 0; i < 6; ++i)
        {
            double phase = computePhase(i, time);
            double x_offset = step_length_ * std::cos(2 * M_PI * phase - M_PI);
            double z_offset = (phase < 0.5) ? step_height_ * std::sin(M_PI * phase * 2) : 0.0;

            foot_positions[i][0] = current_positions_[i][0] + x_offset;
            foot_positions[i][1] = current_positions_[i][1];
            foot_positions[i][2] = current_positions_[i][2] + z_offset;
        }

        return foot_positions;
    }

    void GaitEngine::gaitTypeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        gait_type_ = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Gait type updated to: %s", gait_type_.c_str());
    }

    void GaitEngine::legStateCallback(const hexapod_msgs::msg::LegStateArray::SharedPtr msg)
    {
        if (msg->legs.size() != 6) {
            RCLCPP_WARN(node_->get_logger(), "Received leg_states with size != 6");
            return;
        }

        for (size_t i = 0; i < 6; ++i) {
            current_positions_[i] = {
                msg->legs[i].position.x,
                msg->legs[i].position.y,
                msg->legs[i].position.z
            };
        }

        has_joint_state_ = true;
    }

    

} // namespace hexapod_control
