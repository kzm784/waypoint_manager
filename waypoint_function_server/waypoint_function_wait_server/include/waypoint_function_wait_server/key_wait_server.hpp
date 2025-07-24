#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION__KEY_WAIT_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION__KEY_WAIT_SERVER

#include <waypoint_function_server/function_server_node.hpp>

#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <chrono>

#include <std_msgs/msg/bool.hpp>

namespace waypoint_function 
{
    class KeyWaitServer : public waypoint_function::FunctionServerNode 
    {
        public:
            explicit KeyWaitServer(const rclcpp::NodeOptions & options);
            void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                    std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

        private:
            void callback(const std_msgs::msg::Bool::SharedPtr msg);

            std::string SERVER_NAME    = "key_wait_server";
            std::string COMMAND_HEADER = "key_wait";
            std::string EXECUTE_STATE  = "end";

            bool isWaiting_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION__KEY_WAIT_SERVER