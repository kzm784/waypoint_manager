#include "waypoint_function_example_server/pause_server.hpp"

using namespace std::chrono_literals;

waypoint_function::PauseServer::PauseServer(const rclcpp::NodeOptions &options) : FunctionServerNode("pause_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);
    sub_ = create_subscription<std_msgs::msg::Bool>("/pause_end", 1, std::bind(&PauseServer::callback, this, std::placeholders::_1));
    isWaiting_ = false;
}

void waypoint_function::PauseServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> /*request*/, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> /*response*/)
{
    RCLCPP_INFO(get_logger(), "Pause Server Called.");
    isWaiting_ = true;
}

void waypoint_function::PauseServer::callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data && isWaiting_)
    {
        isWaiting_ = false;
        SendResponse("pause_server:complete");
        return;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::PauseServer)