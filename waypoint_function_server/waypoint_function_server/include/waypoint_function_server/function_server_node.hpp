
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

using namespace std::chrono_literals;

// FunctionServerNode クラス: rclcpp::Node を継承
class FunctionServerNode : public rclcpp::Node 
{
  public:
    explicit FunctionServerNode(const std::string &server_name, const rclcpp::NodeOptions &options);
    virtual void Update(const example_interfaces::msg::Empty::SharedPtr msg);
    virtual void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                    std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
    void SendResponse(std::string result_msg);
    void ServerApply(const std::string &server_name, const std::string &command_header, const std::string &execute_state);

  private:
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr update_sub_;
    rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
    rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr response_pub_;
};
