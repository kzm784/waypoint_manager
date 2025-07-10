#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_WAIT_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_WAIT_SERVER

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

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_WAIT_SERVER