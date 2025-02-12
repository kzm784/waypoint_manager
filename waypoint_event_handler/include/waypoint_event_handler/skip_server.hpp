#ifndef WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__SKIP_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__SKIP_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <waypoint_event_msgs/srv/command.hpp>

namespace waypoint_event 
{
    class SkipServer : public rclcpp::Node
    {
        public:
        explicit SkipServer(const rclcpp::NodeOptions & options);
    
        private:
            void ServerApply();
            void Callback(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response);
        
            rclcpp::Service<waypoint_event_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Client<waypoint_event_msgs::srv::Command>::SharedPtr server_apply_;

            std::string COMMAND_HEADER = "skip";
            std::string SERVER_NAME = "skip_server";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__SKIP_SERVER