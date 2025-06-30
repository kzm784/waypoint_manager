#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

using namespace std;

namespace waypoint_function
{
    class FunctionServer : public rclcpp::Node 
    {
        public:
            explicit FunctionServer(string server_name, string command_header, string execute_state, const rclcpp::NodeOptions &options);
            virtual void Update(const example_interfaces::msg::Empty::SharedPtr msg);
            virtual void Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void SendResponse(string result_msg);
            void ServerApply(string server_name, string command_header, string execute_state);

        private:
            rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr update_sub_;
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr response_pub_;

            string command_header_;
            string server_name_;
            string execute_state;
    };
}