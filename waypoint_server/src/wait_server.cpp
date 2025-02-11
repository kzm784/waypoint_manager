#include "waypoint_server/wait_server.hpp"

using namespace std::chrono_literals;

waypoint_server::WaitServer::WaitServer(const rclcpp::NodeOptions &options) : Node("wait_server", options)
{
    // Create Client For Server Apply
    server_apply_ = create_client<waypoint_server_msgs::srv::ServerInfo>("server_apply");

    // Execute Server Apply
    while (!server_apply_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
    }
    
    auto request = std::make_shared<waypoint_server_msgs::srv::ServerInfo::Request>();
    request->data.push_back(COMMAND_HEADER);
    request->data.push_back(SERVER_NAME);
    auto result = server_apply_->async_send_request(request);

    auto returnCode = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result);
    
    if(returnCode == rclcpp::FutureReturnCode::SUCCESS){
        std::string msg = result.get()->message;
        if(msg[0] == 'S') RCLCPP_INFO(get_logger(), msg.c_str());
        else if (msg[0] == 'F') RCLCPP_ERROR(get_logger(), msg.c_str());
    }
    else RCLCPP_ERROR(get_logger(), "Server Apply Failed.");

    // Create Server
    server_ = create_service<waypoint_server_msgs::srv::ServerInfo>(
        SERVER_NAME,
        std::bind(&WaitServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&WaitServer::Dammy, this)
        );
}
void waypoint_server::WaitServer::Dammy()
{
    RCLCPP_INFO(get_logger(), "Dammy called");
}

void waypoint_server::WaitServer::Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request, std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response)
{
    server_result = response;
    RCLCPP_INFO(get_logger(), "Wait Server Called.");
    int wait_time = std::stof(request->data[1]);
    RCLCPP_INFO(get_logger(), "Wait for %d seconds.", wait_time);
    sleep(wait_time);
    RCLCPP_INFO(get_logger(), "Wait Server Done.");
    SendResponse();
}

void waypoint_server::WaitServer::SendResponse()
{
    server_result->message = "wait_server:complete";
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_server::WaitServer)