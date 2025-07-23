#include "hexapod_teleop/server.hpp"

namespace hexapod_teleop
{
    TeleopServer::TeleopServer() : Node("teleop_server")
    {
        this->declare_parameter("command_type_gait", "gait");
        this->declare_parameter("command_type_speed", "speed");
        this->declare_parameter("command_type_turn", "turn");
        this->declare_parameter("command_type_rotate", "rotate");
        this->declare_parameter("command_type_lift", "lift");

        CMD_GAIT    = this->get_parameter("command_type_gait").as_string();
        CMD_SPEED   = this->get_parameter("command_type_speed").as_string();
        CMD_TURN    = this->get_parameter("command_type_turn").as_string();
        CMD_ROTATE  = this->get_parameter("command_type_rotate").as_string();
        CMD_LIFT    = this->get_parameter("command_type_lift").as_string();

        service_ = this->create_service<hexapod_interfaces::srv::TeleopCommand>(
            "teleop_command",
            std::bind(&TeleopServer::handle_command, this, std::placeholders::_1, std::placeholders::_2));
        
        gait_pub_ = this->create_publisher<std_msgs::msg::String>("/hexapod/gait_type", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::String>("/hexapod/speed", 10);
        direction_pub_ = this->create_publisher<std_msgs::msg::String>("/hexapod/direction", 10);

        RCLCPP_INFO(this->get_logger(), "TeleopServer ready to receive commands");
    }

    void TeleopServer::handle_command(const std::shared_ptr<hexapod_interfaces::srv::TeleopCommand::Request> request,
                                const std::shared_ptr<hexapod_interfaces::srv::TeleopCommand::Response> response)
    {
        std_msgs::msg::String msg;
        msg.data = request->command_value;
        
        if (request->command_type == CMD_GAIT)
            gait_pub_->publish(msg);
        
        else if (request->command_type == CMD_SPEED)
            speed_pub_->publish(msg);
        
        else if (request->command_type == CMD_TURN || request->command_type == CMD_ROTATE || request->command_type == CMD_LIFT)
            direction_pub_->publish(msg);
        
        else {
            response->success = false;
            response->message = "Unknown command type: " + request->command_type;
            return;
        }
    }
}   // namespace hexapod_teleop

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hexapod_teleop::TeleopServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}