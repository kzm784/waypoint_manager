#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_EXAMPLE
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_EXAMPLE

#include <waypoint_function_server/function_server_node.hpp>

namespace waypoint_function
{
  class TestServer : public waypoint_function::FunctionServerNode 
  {
    public:
      TestServer(const rclcpp::NodeOptions &options);
      void Update(const example_interfaces::msg::Empty::SharedPtr msg) override;
      void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
              std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

    private:
      std::string SERVER_NAME    = "test_server";
      std::string COMMAND_HEADER = "test";
      std::string EXECUTE_STATE  = "start";
  };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_EXAMPLE