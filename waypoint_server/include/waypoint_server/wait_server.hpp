#ifndef WAYPOINT_MANAGER__WAYPOINT_SERVER__WAIT_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_SERVER__WAIT_SERVER

#include <rclcpp/rclcpp.hpp>
#include <waypoint_server_msgs/srv/server_info.hpp>

#include <unistd.h>

namespace waypoint_server
{
    class WaitServer : public rclcpp::Node
    {
        public:
        explicit WaitServer(const rclcpp::NodeOptions & options);
    
        private:
        void Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request,
                        std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response);
        void SendResponse();
        void Dammy();
    
        rclcpp::Service<waypoint_server_msgs::srv::ServerInfo>::SharedPtr server_;
        rclcpp::Client<waypoint_server_msgs::srv::ServerInfo>::SharedPtr server_apply_;
        std::shared_ptr<waypoint_server_msgs::srv::ServerInfo_Response> server_result;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string COMMAND_HEADER = "wait";
        std::string SERVER_NAME = "wait_server";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_SERVER__WAIT_SERVER