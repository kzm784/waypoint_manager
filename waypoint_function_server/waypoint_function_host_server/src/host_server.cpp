#include "waypoint_function_host_server/host_server.hpp"

waypoint_function::HostServer::HostServer(const rclcpp::NodeOptions & options) : Node("host_server", options)
{
    RCLCPP_INFO(get_logger(), "Waypoint Host Server Start.");
    
    // Server to Accept Apply and Create Function Server Cilent
    accept_apply_server_ = create_service<waypoint_function_msgs::srv::Command>(
        "server_apply",
        std::bind(&HostServer::ServerApplyAcception, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Publisher to Update Function Servers
    update_executor_ = create_publisher<example_interfaces::msg::Empty>("server_update", 10);

    // Server to Accept Function Commands from Waypoint Navigator
    function_server_ = create_service<waypoint_function_msgs::srv::Command>(
        "function_commands",
        std::bind(&HostServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Subscriber to Recieve Async Response from Function Server 
    function_async_response_ = create_subscription<example_interfaces::msg::String>("async_response", 1,
        bind(&HostServer::ReceiveAsyncResponse, this, std::placeholders::_1));
    
    // Publisher to Send Functions Result to Waypoint Navigator
    function_result_pub_ = create_publisher<example_interfaces::msg::String>("function_result", 10);
}


void waypoint_function::HostServer::ServerApplyAcception(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,const std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    ServerHandle handle_tmp;
    handle_tmp.command_header = request->data[0];
    handle_tmp.server_name = request->data[1]; 
    // Check Server Uniqueness
    for (ServerHandle &handle : server_handles_)
    {
        if(handle_tmp.command_header == handle.command_header)
        {
            response->message = "Falid : Same Command Header Exist.";
            return;
        }
        if(handle_tmp.server_name == handle.server_name)
        {
            response->message = "Falid : Same server Name Exist.";
            return;
        }
    }
    // Create Server Handle
    handle_tmp.client = create_client<waypoint_function_msgs::srv::Command>(handle_tmp.server_name);
    server_handles_.push_back(handle_tmp);

    response->message = "Succes : Server Apply Accepted.";
}


void waypoint_function::HostServer::Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    // Notify Waypoint Updated to Function Servers
    ServerUpdate();

    function_commands_ = request->data;
    commands_size_ = function_commands_.size();

    result_msg_  = "Server Result{ ";
    if(commands_size_ < 1)
    {
        result_msg_ += "Non Server Selected ";
        SendFunctionResults();
    }
    else
    {
        current_command_index_ = 0;
        ServerCall();
    }
}

void waypoint_function::HostServer::ServerCall()
{
    // Split command to elements
    auto command_data = Command_Split(function_commands_[current_command_index_]);

    // Call ServerHandle by Command Header
    for(ServerHandle &handle : server_handles_)
    {
        if(handle.command_header == command_data[0])
        {
            auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
            request->data = command_data;
            // Send Request
            auto future_response = handle.client->async_send_request(
                request, 
                std::bind(&HostServer::ReceiveSyncResponse,this,std::placeholders::_1));
            return;
        }
    }
    result_msg_ += "Wrong Command->" + function_commands_[current_command_index_] + ", " ;
    RunNextCommand();
}

void waypoint_function::HostServer::ReceiveSyncResponse(rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedFuture future)
{
    std::string msg = future.get()->message;
    if(msg == "AsyncResponse") return; // Recieve "AsyncResponse" means FunctionServer Use AsyncResponse 
    result_msg_ += msg + ", ";
    RunNextCommand();
}

void waypoint_function::HostServer::ReceiveAsyncResponse(const example_interfaces::msg::String::SharedPtr msg)
{
    result_msg_ += msg->data + ", ";
    RunNextCommand();
}

void waypoint_function::HostServer::RunNextCommand()
{
    // Update command_index and Check if all-command were runned  
    current_command_index_++;
    if(current_command_index_ < commands_size_) ServerCall();
    else SendFunctionResults();
}

void waypoint_function::HostServer::SendFunctionResults()
{
    result_msg_ += "}";
    // Send Function Results to Waypoint Navigator
    example_interfaces::msg::String msg;
    msg.data = result_msg_;
    function_result_pub_->publish(msg);
}


void waypoint_function::HostServer::ServerUpdate()
{
    example_interfaces::msg::Empty msg;
    update_executor_->publish(msg);
}


std::vector<std::string> waypoint_function::HostServer::Command_Split(std::string &command_line)
{
    // Split command_line by ":"
    std::vector<std::string> command;
    std::stringstream ss(command_line);
    std::string item;
    while (std::getline(ss, item, ':')) {
        command.push_back(item);
    }
    return command;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::HostServer)