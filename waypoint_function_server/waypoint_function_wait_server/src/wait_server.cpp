#include "waypoint_function_wait_server/wait_server.hpp"

using namespace std::chrono_literals;

waypoint_function::WaitServer::WaitServer(const rclcpp::NodeOptions &options) : FunctionServerNode("wait_server_node", options) 
{
  ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);
}

void waypoint_function::WaitServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Wait Server Called.");
    int wait_time = std::stof(request->data[1]);
    response->message = "AsyncResponse";
    TimeCount(wait_time);
}

void waypoint_function::WaitServer::TimeCount(int time)
{
    RCLCPP_INFO(get_logger(), "Wait for %d seconds.", time);
    for(int i=time; i>0; i--)
    {
        std::cout << i << std::endl;
        sleep(1);
    }
    RCLCPP_INFO(get_logger(), "Wait Server Done.");
    SendResponse("wait_server:complete");
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::WaitServer)