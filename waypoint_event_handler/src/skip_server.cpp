#include "waypoint_event_handler/skip_server.hpp"

using namespace std::chrono_literals;

waypoint_event::SkipServer::SkipServer(const rclcpp::NodeOptions &options) : Node("skip_server", options)
{
    // Create Server
    server_ = create_service<waypoint_event_msgs::srv::Command>(
        SERVER_NAME,
        std::bind(&SkipServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create Client For Server Apply
    server_apply_ = create_client<waypoint_event_msgs::srv::Command>("server_apply");
    // Apply Tihs Server to create connection to Host Server
    ServerApply();
}

void waypoint_event::SkipServer::ServerApply()
{
    // Execute Server Apply
    while (!server_apply_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
    }
    
    auto request = std::make_shared<waypoint_event_msgs::srv::Command::Request>();
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

}

void waypoint_event::SkipServer::Callback(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response)
{
    // RCLCPP_INFO(get_logger(), "Skip Server Called.");

    // RCLCPP_INFO(get_logger(), "Skip Server Done.");

    response->message = "skip_server:complete";
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_event::SkipServer)
