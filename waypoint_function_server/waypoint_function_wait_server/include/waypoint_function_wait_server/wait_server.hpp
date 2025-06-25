#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_WAIT_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_WAIT_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

#include <unistd.h>

using namespace std;

namespace waypoint_function 
{
    class WaitServer : public rclcpp::Node
    {
        public:
            explicit WaitServer(const rclcpp::NodeOptions & options);

        private:
            void ServerApply();
            void Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void Delay(int sleep_time);
        
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr server_apply_;
            rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr response_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            string COMMAND_HEADER = "wait";
            string SERVER_NAME = "wait_server";
            string EXECUTE_STATE = "start";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_WAIT_SERVER