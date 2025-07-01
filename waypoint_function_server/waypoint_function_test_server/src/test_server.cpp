#include "waypoint_function_test_server/test_server.hpp"

TestServer::TestServer( const rclcpp::NodeOptions& options): FunctionServerNode("test_server_node", options) 
{
  ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);

  // timer_ = this->create_wall_timer(
  //   1000ms, std::bind(&TestServer::timer_callback, this));
}

void TestServer::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Hello from TestServer!");
}

void TestServer::Update(const example_interfaces::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Update Called");
}

void TestServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
        std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Test Server Callback awake");
    std::string msg = "test_server:complete";
    response->message = "test:complete";
    // SendResponse(msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestServer)
