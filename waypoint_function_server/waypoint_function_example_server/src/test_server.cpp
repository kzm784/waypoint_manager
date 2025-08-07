#include "waypoint_function_example_server/test_server.hpp"

waypoint_function::TestServer::TestServer( const rclcpp::NodeOptions& options) : FunctionServerNode("test_server_node", options) 
{
  ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);
}

void waypoint_function::TestServer::Update(const std_msgs::msg::Empty::SharedPtr)
{
    RCLCPP_INFO(this->get_logger(), "Update Called");
}

void waypoint_function::TestServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> /*request*/, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> /*response*/)
{
    RCLCPP_INFO(this->get_logger(), "Test Server Callback awake");
    std::string result_msg = "test_server:complete";
    SendResponse(result_msg);
    // response->message = msg;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::TestServer)
