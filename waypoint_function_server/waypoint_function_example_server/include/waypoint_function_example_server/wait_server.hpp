#ifndef WAYPOINT_FUNCTION_SERVER__WAYPOINT_FUNCTION_EXAMPLE_SERVER__WAIT_SERVER_HPP_
#define WAYPOINT_FUNCTION_SERVER__WAYPOINT_FUNCTION_EXAMPLE_SERVER__WAIT_SERVER_HPP_

#include <waypoint_function_server/function_server_node.hpp>

#include <unistd.h>

namespace waypoint_function 
{
    class WaitServer : public waypoint_function::FunctionServerNode 
    {
        public:
            explicit WaitServer(const rclcpp::NodeOptions & options);
            void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                    std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

        private:
            void TimeCount(int time);

            std::string SERVER_NAME    = "wait_server";
            std::string COMMAND_HEADER = "wait";
            std::string EXECUTE_STATE  = "end";
    };
}

#endif  // WAYPOINT_FUNCTION_SERVER__WAYPOINT_FUNCTION_EXAMPLE_SERVER__WAIT_SERVER_HPP_