#include "waypoint_function_host_server/host_server.hpp"

waypoint_function::HostServer::HostServer(const rclcpp::NodeOptions & options) : Node("host_server", options)
{
    RCLCPP_INFO(get_logger(), "Waypoint Host Server Start.");
    
    // Server to Accept Apply and Create Function Server Cilent
    accept_apply_server_ = create_service<waypoint_function_msgs::srv::Command>(
        "server_apply",
        bind(&HostServer::ServerApplyAcception, this, placeholders::_1, placeholders::_2)
    );

    // Publisher to Update Function Servers
    update_executor_ = create_publisher<example_interfaces::msg::Empty>("server_update", 10);

    // Server to Accept Function Commands from Waypoint Navigator
    function_server_ = create_service<waypoint_function_msgs::srv::Command>(
        "function_commands",
        bind(&HostServer::Callback, this, placeholders::_1, placeholders::_2)
    );
    // Subscriber to Recieve Async Response from Function Server 
    function_async_response_ = create_subscription<example_interfaces::msg::String>("async_response", 1,
        bind(&HostServer::ReceiveAsyncResponse, this, placeholders::_1));
    
    // Publisher to Send Functions Result to Waypoint Navigator
    function_result_pub_ = create_publisher<example_interfaces::msg::String>("function_results", 10);
    function_results_client_ = create_client<waypoint_function_msgs::srv::Command>("function_results");
}


void waypoint_function::HostServer::ServerApplyAcception(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,const shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    ServerHandle handle_tmp;
    handle_tmp.command_header = request->data[0];
    handle_tmp.server_name = request->data[1]; 
    handle_tmp.execute_state = request->data[2];
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


void waypoint_function::HostServer::Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request, shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    execute_state_ = request->execute_state;
    function_commands_ = request->data;
    commands_size_ = function_commands_.size();

    // Notify Waypoint Updated to Function Servers
    if(execute_state_ == "start") ServerUpdate();

    results_msg_ = {};
    results_msg_.push_back("===Results===" + execute_state_);
    if(commands_size_ < 1)
    {
        results_msg_.push_back(" Non Server Selected ");
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
        if(handle.command_header == command_data[0] && handle.execute_state == execute_state_)
        {
            auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
            request->data = command_data;
            // Send Request
            auto future_response = handle.client->async_send_request(
                request, 
                bind(&HostServer::ReceiveSyncResponse,this,placeholders::_1));
            return;
        }
    }
    RunNextCommand();
}

void waypoint_function::HostServer::ReceiveSyncResponse(rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedFuture future)
{
    string msg = future.get()->message;
    if(msg == "AsyncResponse" || msg == "") return; // when Recieve "AsyncResponse" or null message "", Use AsyncResponse 
    results_msg_.push_back(msg);
    RunNextCommand();
}

void waypoint_function::HostServer::ReceiveAsyncResponse(const example_interfaces::msg::String::SharedPtr msg)
{
    results_msg_.push_back(msg->data);
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
    results_msg_.push_back("=============");

    // Send Function Results to Waypoint Navigator
    auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
    request->data = results_msg_;
    request->execute_state = execute_state_;
    while (!function_results_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
      }
    function_results_client_->async_send_request(request);
}


void waypoint_function::HostServer::ServerUpdate()
{
    example_interfaces::msg::Empty msg;
    update_executor_->publish(msg);
}


vector<string> waypoint_function::HostServer::Command_Split(string &command_line)
{
    // Split command_line by ":"
    vector<string> command;
    stringstream ss(command_line);
    string item;
    while (getline(ss, item, ':')) {
        command.push_back(item);
    }
    return command;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::HostServer)