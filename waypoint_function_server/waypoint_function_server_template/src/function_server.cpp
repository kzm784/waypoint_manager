#include "waypoint_function_server_template/function_server.hpp"

waypoint_function::FunctionServer::FunctionServer(string server_name, string command_header, string execute_state, const rclcpp::NodeOptions &options) : Node(server_name, options)
{
    // Subscriber to update when waypoint updated
    update_sub_ = create_subscription<example_interfaces::msg::Empty>("server_update", 1,
        bind(&FunctionServer::Update, this, std::placeholders::_1));
    
    // Create Server
    server_ = create_service<waypoint_function_msgs::srv::Command>(
        server_name,
        std::bind(&FunctionServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    response_pub_ = create_publisher<example_interfaces::msg::String>("async_response", 10);

    // Client for Server Apply
    apply_client_ = create_client<waypoint_function_msgs::srv::Command>("server_apply");
    // Apply Tihs Server to Host Server to make connection
    ServerApply(server_name, command_header, execute_state);
}

waypoint_function::FunctionServer::SendResponse(string result_msg)
{
    response_pub_->publish(result_msg);
}

waypoint_function::FunctionServer::ServerApply(string server_name, string command_header, string execute_state)
{
    // Execute Server Apply
    while (!apply_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
    }
    
    // Send Request to Host Server
    auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
    request->data.push_back(command_header);
    request->data.push_back(server_name);
    request->data.push_back(execute_state);
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