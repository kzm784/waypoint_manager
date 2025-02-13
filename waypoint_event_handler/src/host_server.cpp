#include "waypoint_event_handler/host_server.hpp"

waypoint_event::HostServer::HostServer(const rclcpp::NodeOptions & options) : Node("host_server", options)
{
    RCLCPP_INFO(get_logger(), "Waypoint Host Server Start.");
    
    // Server to Create Event Cilent
    apply_server_ = create_service<waypoint_event_msgs::srv::Command>(
        "server_apply",
        std::bind(&HostServer::ServerApplyAcception, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Publisher to Update Servers
    update_executor_ = create_publisher<example_interfaces::msg::Empty>("server_update", 10);

    // Server to Accept Event Commands
    event_server_ = create_service<waypoint_event_msgs::srv::Command>(
        "event_commands",
        std::bind(&HostServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    // Subscriber to Recieve Async Response
    event_async_response_ = create_subscription<example_interfaces::msg::String>("async_response", 1,
        bind(&HostServer::ReceiveAsyncResponse, this, std::placeholders::_1));
    // Publisher to Send Event Result
    event_result_pub_ = create_publisher<example_interfaces::msg::String>("event_result", 10);
}


void waypoint_event::HostServer::ServerApplyAcception(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request,const std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response)
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
    handle_tmp.client = create_client<waypoint_event_msgs::srv::Command>(handle_tmp.server_name);
    server_handles_.push_back(handle_tmp);

    response->message = "Succes : Server Apply Accepted.";
}


void waypoint_event::HostServer::Callback(const std::shared_ptr<waypoint_event_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_event_msgs::srv::Command::Response> response)
{
    ServerUpdate();

    server_commands = request->data;
    commands_size = server_commands.size();

    result_msg  = "Server Result{ ";
    if(commands_size < 1)
    {
        result_msg += "Non Server Selected ";
        SendEventResults();
    }
    else
    {
        command_num = 0;
        ServerCall(server_commands[command_num]);
    }
}

void waypoint_event::HostServer::ServerCall(std::string &command)
{
    auto command_data = Command_Split(command);
    for(ServerHandle &handle : server_handles_)
    {
        if(handle.command_header == command_data[0])
        {
            auto request = std::make_shared<waypoint_event_msgs::srv::Command::Request>();
            request->data = command_data;
        
            auto future_response = handle.client->async_send_request(
                request, 
                std::bind(&HostServer::ReceiveSyncResponse,this,std::placeholders::_1));
            return;
        }
    }
    result_msg += "Wrong Command->" + command + ", " ;
    command_num++;
    if(command_num < commands_size) ServerCall(server_commands[command_num]);
    else SendEventResults();
}

void waypoint_event::HostServer::ReceiveSyncResponse(rclcpp::Client<waypoint_event_msgs::srv::Command>::SharedFuture future)
{
    std::string msg = future.get()->message;
    if(msg == "AsyncResponse") return;
    result_msg += msg + ", ";
    command_num++;
    if(command_num < commands_size) ServerCall(server_commands[command_num]);
    else SendEventResults();
}

void waypoint_event::HostServer::ReceiveAsyncResponse(const example_interfaces::msg::String::SharedPtr msg)
{
    result_msg += msg->data + ", ";
    command_num++;
    if(command_num < commands_size) ServerCall(server_commands[command_num]);
    else SendEventResults();
}

void waypoint_event::HostServer::SendEventResults()
{
    result_msg += "}";
    // RCLCPP_INFO(get_logger(), result_msg.c_str());
    example_interfaces::msg::String msg;
    msg.data = result_msg;
    event_result_pub_->publish(msg);
}


void waypoint_event::HostServer::ServerUpdate()
{
    example_interfaces::msg::Empty msg;
    update_executor_->publish(msg);
}


std::vector<std::string> waypoint_event::HostServer::Command_Split(std::string command_line)
{
    std::vector<std::string> command;
    std::stringstream ss(command_line);
    std::string item;
    while (std::getline(ss, item, ':')) {
        command.push_back(item);
    }
    return command;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_event::HostServer)