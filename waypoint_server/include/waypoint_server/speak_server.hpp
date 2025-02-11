#ifndef WAYPOINT_MANAGER__WAYPOINT_SERVER__SPEAK_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_SERVER__SPEAK_SERVER

#include <rclcpp/rclcpp.hpp>
#include <waypoint_server_msgs/srv/server_info.hpp>

namespace waypoint_server
{
    class SpeakServer : public rclcpp::Node
    {
        public:
        explicit SpeakServer(const rclcpp::NodeOptions & options);
    
        private:
            void Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request,
                            std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response);
        
            rclcpp::Service<waypoint_server_msgs::srv::ServerInfo>::SharedPtr server_;
            rclcpp::Client<waypoint_server_msgs::srv::ServerInfo>::SharedPtr server_apply_;

            std::string COMMAND_HEADER = "speak";
            std::string SERVER_NAME = "speak_server";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_SERVER__SPEAK_SERVER