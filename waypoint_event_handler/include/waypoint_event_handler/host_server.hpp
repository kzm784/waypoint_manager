#ifndef WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__HOST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__HOST_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_event_msgs/srv/command.hpp>

namespace waypoint_event 
{
    class ServerHandle
    {
        public:
            std::string command_header;
            std::string server_name;
            std::shared_ptr<rclcpp::Client<waypoint_event_msgs::srv::Command>> client;
    };

    class HostServer : public rclcpp::Node
    {
        public:
            explicit HostServer(const rclcpp::NodeOptions & options);
        
        private:
            void ServerApplyAcception(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response);
            void Callback(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response);
            void ServerCall(std::string &command);
            void ReceiveSyncResponse(rclcpp::Client<waypoint_event_msgs::srv::Command>::SharedFuture future);
            void ReceiveAsyncResponse(const example_interfaces::msg::String::SharedPtr msg);
            void SendEventResults();
            void ServerUpdate();
            std::vector<std::string> Command_Split(std::string command_line);

            rclcpp::Service<waypoint_event_msgs::srv::Command>::SharedPtr apply_server_;
            rclcpp::CallbackGroup::SharedPtr client_cb_group_;
            rclcpp::Service<waypoint_event_msgs::srv::Command>::SharedPtr event_server_;
            rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr event_async_response_;
            rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr event_result_pub_;
            rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr update_executor_;

            std::vector<ServerHandle> server_handles_ ;
            std::vector<std::string> server_commands;
            int commands_size = 0, command_num = 0;
            std::string result_msg = "";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__HOST_SERVER