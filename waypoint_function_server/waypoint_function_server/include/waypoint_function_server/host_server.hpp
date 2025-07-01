#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_HOST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_HOST_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

#include "server_handle.hpp"
// #include <waypoint_function_server/server_handle.hpp>

using namespace std;

namespace waypoint_function 
{
    class HostServer : public rclcpp::Node
    {
        public:
            explicit HostServer(const rclcpp::NodeOptions & options);
        
        private:
            void ServerApplyAcception(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void ServerCall();
            void ReceiveSyncResponse(rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedFuture future);
            void ReceiveAsyncResponse(const example_interfaces::msg::String::SharedPtr msg);
            void RunNextCommand();
            void SendFunctionResults();
            void ServerUpdate();
            vector<string> Command_Split(string &command_line);

            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr accept_apply_server_;
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr function_server_;
            rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr function_async_response_;
            rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr function_result_pub_;
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr function_results_client_;
            rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr update_executor_;

            vector<waypoint_function::ServerHandle> server_handles_ ;
            vector<string> function_commands_;
            string execute_state_;
            int commands_size_ = 0, current_command_index_ = 0;
            vector<string> results_msg_;
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_HOST_SERVER