#ifndef WAYPOINT_MANAGER__WAYPOINT_SERVER__MAP_CHANGE_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_SERVER__MAP_CHANGE_SERVER

#include <rclcpp/rclcpp.hpp>
#include <waypoint_server_msgs/srv/server_info.hpp>

namespace waypoint_server
{
    class MapChangeServer : public rclcpp::Node
    {
        public:
        explicit MapChangeServer(const rclcpp::NodeOptions & options);
    
        private:
            void Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request,
                            std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response);
            rclcpp::Service<waypoint_server_msgs::srv::ServerInfo>::SharedPtr server_;
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_SERVER__MAP_CHANGE_SERVER