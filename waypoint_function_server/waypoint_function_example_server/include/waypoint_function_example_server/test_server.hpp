#ifndef WAYPOINT_FUNCTION_SERVER__WAYPOINT_FUNCTION_EXAMPLE_SERVER__TEST_SERVER_HPP_
#define WAYPOINT_FUNCTION_SERVER__WAYPOINT_FUNCTION_EXAMPLE_SERVER__TEST_SERVER_HPP_

#include <waypoint_function_server/function_server_node.hpp>

namespace waypoint_function
{
  class TestServer : public waypoint_function::FunctionServerNode 
  {
    public:
      TestServer(const rclcpp::NodeOptions &options);
      void Update(const std_msgs::msg::Empty::SharedPtr) override;
      void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
              std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

    private:
      std::string SERVER_NAME    = "test_server";
      std::string COMMAND_HEADER = "test";
      std::string EXECUTE_STATE  = "start";
  };
}

#endif  // WAYPOINT_FUNCTION_SERVER__WAYPOINT_FUNCTION_EXAMPLE_SERVER__TEST_SERVER_HPP_