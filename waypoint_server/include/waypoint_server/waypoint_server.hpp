#ifndef WAYPOINT_MANAGER__WAYPOINT_SERVER_
#define WAYPOINT_MANAGER__WAYPOINT_SERVER_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_server_msgs/srv/command.hpp>

class WaypointServer : public rclcpp::Node
{
    public:
        explicit WaypointServer(const rclcpp::NodeOptions & options);
    
    private:
        void Callback(const std::shared_ptr<waypoint_server_msgs::srv::Command::Request> request,
                        std::shared_ptr<waypoint_server_msgs::srv::Command::Response> response);
        void ServerUpdate();
        void ServerCall(std::string command_line);
        std::vector<std::string> Command_Split(std::string command_line);

        rclcpp::Service<waypoint_server_msgs::srv::Command>::SharedPtr server_;
        rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr update_executor_;
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_SERVER_