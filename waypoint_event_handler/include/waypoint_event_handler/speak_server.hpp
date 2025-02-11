#ifndef WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__SPEAK_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__SPEAK_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <waypoint_event_msgs/srv/command.hpp>

namespace waypoint_event 
{
    class SpeakServer : public rclcpp::Node
    {
        public:
        explicit SpeakServer(const rclcpp::NodeOptions & options);
    
        private:
            void ServerApply();
            void Callback(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response);
        
            rclcpp::Service<waypoint_event_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Client<waypoint_event_msgs::srv::Command>::SharedPtr server_apply_;

            std::string COMMAND_HEADER = "speak";
            std::string SERVER_NAME = "speak_server";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__SPEAK_SERVER