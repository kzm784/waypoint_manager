
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

using namespace std::chrono_literals;

namespace waypoint_function
{
  // FunctionServerNode クラス: rclcpp::Node を継承
  class FunctionServerNode : public rclcpp::Node 
  {
    public:
      explicit FunctionServerNode(const std::string &server_name, const rclcpp::NodeOptions &options);
      virtual void Update(const std_msgs::msg::Empty::SharedPtr);
      virtual void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                      std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
      void SendResponse(std::string result_msg);
      void ServerApply(const std::string &server_name, const std::string &command_header, const std::string &execute_state);

    private:
      rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_sub_;
      rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delete_apply_pub_;
      rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;

      std::string server_name_;
  };
} // namespace waypoint_function

