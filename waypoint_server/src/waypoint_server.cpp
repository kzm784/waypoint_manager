#include "waypoint_server/waypoint_server.hpp"

WaypointServer::WaypointServer(const rclcpp::NodeOptions & options) : Node("waypoint_server", options)
{
    RCLCPP_INFO(get_logger(), "Waypoint Server Start.");

    server_ = create_service<waypoint_server_msgs::srv::Command>(
        "waypoint_command",
        std::bind(&WaypointServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    update_executor_ = create_publisher<example_interfaces::msg::Empty>("server_update", 10);
}


void WaypointServer::Callback(const std::shared_ptr<waypoint_server_msgs::srv::Command::Request> request, 
                    std::shared_ptr<waypoint_server_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Accepted Server Command");
    for ( std::string command_line : request->commands )
    {
        ServerCall(command_line);
    }

    response->message = "result_msg";
}


void WaypointServer::ServerUpdate()
{
    RCLCPP_INFO(get_logger(), "Server Update");
    example_interfaces::msg::Empty msg;
    update_executor_->publish(msg);
}


void WaypointServer::ServerCall(std::string command_line)
{
    std::vector<std::string> command = Command_Split(command_line);

    std::string result_msg;

    if (command[0] == "update")
    {
        ServerUpdate();
    }
    else if (command[0] == "wait")
    {
        RCLCPP_INFO(get_logger(), "Wait Server Call");
    }
    else if (command[0] == "skip")
    {
        RCLCPP_INFO(get_logger(), "Skip Server Call");
    }
    else if (command[0] == "speak")
    {
        RCLCPP_INFO(get_logger(), "Speak Server Call");
    }
    else if (command[0] == "map_change")
    {
        RCLCPP_INFO(get_logger(), "Map Change Server Call");
    }
    else 
    {
        RCLCPP_WARN(get_logger(), "Wrong Command entered");
    }
}
std::vector<std::string> WaypointServer::Command_Split(std::string command_line)
{
    std::vector<std::string> command;
    std::string tmp_str = "";
    for (char c : command_line)
    {
        if (c == ':')
        {
            command.push_back(tmp_str);
            tmp_str = "";
        }
        else 
        {
            tmp_str += c;
        }
    }
    command.push_back(tmp_str);

    return command;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(WaypointServer)