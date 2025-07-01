#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER

#include <chrono>
#include <memory>
#include <string>

#include <waypoint_function_server/function_server_node.hpp>

using namespace std::chrono_literals;

// TestServer クラス: FunctionServerNode を継承し、文字列を Publish
class TestServer : public FunctionServerNode 
{
  public:
    TestServer(const rclcpp::NodeOptions &options);
    void Update(const example_interfaces::msg::Empty::SharedPtr msg) override;
    void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
            std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string SERVER_NAME    = "test_server";
    std::string COMMAND_HEADER = "test";
    std::string EXECUTE_STATE  = "start";
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER