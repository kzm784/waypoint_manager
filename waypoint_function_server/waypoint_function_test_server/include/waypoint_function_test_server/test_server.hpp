#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// NodeBase クラス: rclcpp::Node を継承
class NodeBase : public rclcpp::Node {
public:
  explicit NodeBase(const std::string & node_name, const rclcpp::NodeOptions& options);
};

// TestNode クラス: NodeBase を継承し、文字列を Publish
class TestNode : public NodeBase {
public:
  TestNode( const rclcpp::NodeOptions& options);

private:
  void timer_callback();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


// #include <waypoint_function_test_server/function_server.hpp>

// #include <unistd.h>

// using namespace std;

// namespace waypoint_function
// {
//     class TestServer : public waypoint_function::FunctionServer
//     {
//         public:
//             explicit TestServer(const rclcpp::NodeOptions &options);
//             void Update(const example_interfaces::msg::Empty::SharedPtr msg) override;
//             void Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
//                     shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

//         private:
//             string COMMAND_HEADER = "test";
//             string SERVER_NAME = "test_server";
//             string EXECUTE_STATE = "start";
//     };
// }

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER