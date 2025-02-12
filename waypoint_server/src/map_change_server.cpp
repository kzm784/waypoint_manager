#include "waypoint_server/map_change_server.hpp"

waypoint_server::MapChangeServer::MapChangeServer(const rclcpp::NodeOptions &options) : Node("map_change_server", options)
{
    RCLCPP_INFO(get_logger(), "MapChange Server Start.");

    server_ = create_service<waypoint_server_msgs::srv::ServerInfo>(
        "waipoint_server/map_change_server",
        std::bind(&MapChangeServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void waypoint_server::MapChangeServer::Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request, std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response)
{
    RCLCPP_INFO(get_logger(), "MapChange Server Called.");

    RCLCPP_INFO(get_logger(), "MapChange Server Done.");

    response->message = "map_change_server:complete";
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_server::MapChangeServer)
