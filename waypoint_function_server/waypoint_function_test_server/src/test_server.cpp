#include "waypoint_function_test_server/test_server.hpp"

waypoint_function::TestServer::TestServer(const rclcpp::NodeOptions &options) : FunctionServer(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE, options)
{
    RCLCPP_INFO(this->get_logger(), "Test Server Start.");
}

void waypoint_function::TestServer::Update() override
{
    RCLCPP_INFO(this->get_logger(), "Update Called");
}

void waypoint_function::TestServer::Callback() override
{
    RCLCPP_INFO(this->get_logger(), "Test Server Callback awake");
    string msg = "test_server:complete";
    SendResponse(msg);
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::TestServer)
