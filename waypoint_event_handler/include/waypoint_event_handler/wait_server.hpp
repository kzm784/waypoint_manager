#ifndef WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__WAIT_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__WAIT_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <waypoint_event_msgs/srv/command.hpp>

#include <unistd.h>

namespace waypoint_event 
{
    class WaitServer : public rclcpp::Node
    {
        public:
            explicit WaitServer(const rclcpp::NodeOptions & options);

        private:
            void ServerApply();
            void Callback(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response);
            void Delay(int sleep_time);
        
            rclcpp::Service<waypoint_event_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Client<waypoint_event_msgs::srv::Command>::SharedPtr server_apply_;
            rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr response_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::string COMMAND_HEADER = "wait";
            std::string SERVER_NAME = "wait_server";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_EVENT_HANDLER__WAIT_SERVER