#ifndef WAYPOINT_MANAGER__WAYPOINT_SERVER__HOST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_SERVER__HOST_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_server_msgs/srv/server_info.hpp>

namespace waypoint_server 
{
    class ServerHandle
    {
        public:
            std::string command_header;
            std::string server_name;
            std::shared_ptr<rclcpp::Client<waypoint_server_msgs::srv::ServerInfo>> client;
    };

    class HostServer : public rclcpp::Node
    {
        public:
            explicit HostServer(const rclcpp::NodeOptions & options);
        
        private:
            void ServerApplyAcception(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request,
                            std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response);
            void Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request,
                            std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response);
            void ServerCall(std::string &command);
            void RunCommands(rclcpp::Client<waypoint_server_msgs::srv::ServerInfo>::SharedFuture future);
            void SendResponse();
            void ServerUpdate();
            std::vector<std::string> Command_Split(std::string command_line);

            rclcpp::Service<waypoint_server_msgs::srv::ServerInfo>::SharedPtr apply_server_;
            rclcpp::CallbackGroup::SharedPtr client_cb_group_;
            rclcpp::Service<waypoint_server_msgs::srv::ServerInfo>::SharedPtr host_server_;
            std::shared_ptr<waypoint_server_msgs::srv::ServerInfo_Response> server_result;
            rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr update_executor_;

            std::vector<ServerHandle> server_handles_;
            std::vector<std::string> server_commands;
            int commands_size = 0, command_num = 0;
            std::string result_msg = "";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_SERVER__HOST_SERVER