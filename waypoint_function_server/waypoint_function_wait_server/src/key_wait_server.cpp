#include "waypoint_function_wait_server/key_wait_server.hpp"

using namespace std::chrono_literals;

waypoint_function::KeyWaitServer::KeyWaitServer(const rclcpp::NodeOptions &options) : FunctionServerNode("wait_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);
    sub_ = create_subscription<example_interfaces::msg::Bool>("/eStop", 1, std::bind(&KeyWaitServer::callback, this, std::placeholders::_1));
    isWaiting_ = false;
}

void waypoint_function::KeyWaitServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Key Wait Server Called.");
    isWaiting_ = true;
}

void waypoint_function::KeyWaitServer::callback(const example_interfaces::msg::Bool::SharedPtr msg)
{
    if(!(msg->data) && isWaiting_)
    {
        isWaiting_ = false;
        SendResponse("key_server:complete");
        return;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::KeyWaitServer)