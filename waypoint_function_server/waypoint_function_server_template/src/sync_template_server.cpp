#include "waypoint_function_server_template/sync_template_server.hpp"

using namespace std::chrono_literals;

waypoint_function::SyncTemplateServer::SyncTemplateServer(const rclcpp::NodeOptions &options) : Node("sync_template_server", options)
{
    // Subscriber to update when waypoint updated
    update_sub_ = create_subscription<example_interfaces::msg::Empty>("server_update", 1,
        bind(&SyncTemplateServer::Update, this, std::placeholders::_1));
    
    // Create Server
    server_ = create_service<waypoint_function_msgs::srv::Command>(
        SERVER_NAME,
        std::bind(&SyncTemplateServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create Client for Server Apply
    apply_client_ = create_client<waypoint_function_msgs::srv::Command>("server_apply");
    // Apply Tihs Server to Host Server to create connection
    ServerApply();
}

void waypoint_function::SyncTemplateServer::Update(const example_interfaces::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Sync Template Server Update.");
    /* write upate code when waypoint updated */
}

void waypoint_function::SyncTemplateServer::Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Sync Template Server Called.");
    // Write Result 
    std::string result_msg ;
    result_msg = "complete"; // Result Message Example 1
    result_msg = "recieve(" + request->data[1] + "&" + request->data[2] +")"; // Result Message Example 2
    // Send Result Message
    response->message = SERVER_NAME + ":" + result_msg;
}

void waypoint_function::SyncTemplateServer::ServerApply()
{
    // Execute Server Apply
    while (!apply_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. sync_templateing...");
    }
    
    // Send Request to Host Server
    auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
    request->data.push_back(COMMAND_HEADER);
    request->data.push_back(SERVER_NAME);
    auto result = apply_client_->async_send_request(request);

    auto returnCode = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result);
    
    // Wait for Recieving Result
    if(returnCode == rclcpp::FutureReturnCode::SUCCESS){
        std::string msg = result.get()->message;
        if(msg[0] == 'S') RCLCPP_INFO(get_logger(), msg.c_str());
        else if (msg[0] == 'F') RCLCPP_ERROR(get_logger(), msg.c_str());
    }
    else RCLCPP_ERROR(get_logger(), "Server Apply Failed.");

}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::SyncTemplateServer)