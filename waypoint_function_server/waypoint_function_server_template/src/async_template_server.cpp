#include "waypoint_function_server_template/async_template_server.hpp"

using namespace std::chrono_literals;

/*--<Change Name>--------------------------------------*/
/*---Package Name : waypoint_function_server_template--*/
/*---Class Name : AsyncTemplateServer------------------*/
/*---Server Name : async_template----------------------*/
/*-----------------------------------------------------*/

waypoint_function::AsyncTemplateServer::AsyncTemplateServer(const rclcpp::NodeOptions &options) : Node("async_template_server", options)
{
    // Subscriber to update when waypoint updated
    update_sub_ = create_subscription<example_interfaces::msg::Empty>("server_update", 1,
        bind(&AsyncTemplateServer::Update, this, std::placeholders::_1));
    
    // Create Server
    server_ = create_service<waypoint_function_msgs::srv::Command>(
        SERVER_NAME,
        std::bind(&AsyncTemplateServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    response_pub_ = create_publisher<example_interfaces::msg::String>("async_response", 10);

    // Client for Server Apply
    apply_client_ = create_client<waypoint_function_msgs::srv::Command>("server_apply");
    // Apply Tihs Server to Host Server to make connection
    ServerApply();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&AsyncTemplateServer::TimerCallback, this));
    _isAvalable = false;
}

void waypoint_function::AsyncTemplateServer::Update(const example_interfaces::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Async Template Server Update.");//Change here
    /*----------------------------------------*/
    /*-write upate code when waypoint updated-*/
    /*----------------------------------------*/
}

void waypoint_function::AsyncTemplateServer::Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, 
    std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Async Template Server Called.");//Change here
    _isAvalable = true;
    // Send message to Host Server to Notify Using Async Response 
    response->message = "AsyncResponse";
}

void waypoint_function::AsyncTemplateServer::TimerCallback()
{
    if(_isAvalable)
    {
        /*------------------------*/
        /*-----write code here----*/
        /*------------------------*/

        SendAsyncResponse();
    }
}

void waypoint_function::AsyncTemplateServer::SendAsyncResponse()
{
    _isAvalable = false;

    // Write Result 
    std::string result_msg ;
    result_msg = "complete"; // Result Message Example 

    // Send Result Message
    example_interfaces::msg::String msg;
    msg.data = SERVER_NAME + ":" + result_msg;
    response_pub_->publish(msg);
}

void waypoint_function::AsyncTemplateServer::ServerApply()
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
    request->data.push_back(COMMAND_HEADER);
    request->data.push_back(SERVER_NAME);
    request->data.push_back(EXECUTE_STATE);
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
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::AsyncTemplateServer)