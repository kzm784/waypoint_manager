#include "waypoint_function_test_server/test_server.hpp"

NodeBase::NodeBase(const std::string &node_name,  const rclcpp::NodeOptions& options): Node(node_name, options) {
    RCLCPP_INFO(this->get_logger(), "NodeBase initialized with name: %s", node_name.c_str());
  }

TestNode::TestNode( const rclcpp::NodeOptions& options): NodeBase("test_node", options) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TestNode::timer_callback, this));
  }

  void TestNode::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello from TestNode!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

// waypoint_function::TestServer::TestServer(const rclcpp::NodeOptions &options) : FunctionServer(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE, options)
// {
//     RCLCPP_INFO(this->get_logger(), "Test Server Start.");
// }

// void waypoint_function::TestServer::Update(const example_interfaces::msg::Empty::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(), "Update Called");
// }

// void waypoint_function::TestServer::Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
//         shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
// {
//     RCLCPP_INFO(this->get_logger(), "Test Server Callback awake");
//     string msg = "test_server:complete";
//     SendResponse(msg);
// }
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestNode)
