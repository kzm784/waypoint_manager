#include "waypoint_server/host_server.hpp"

waypoint_server::HostServer::HostServer(const rclcpp::NodeOptions & options) : Node("host_server", options)
{
    RCLCPP_INFO(get_logger(), "Waypoint Host Server Start.");
    
    apply_server_ = create_service<waypoint_server_msgs::srv::ServerInfo>(
        "server_apply",
        std::bind(&HostServer::ServerApplyAcception, this, std::placeholders::_1, std::placeholders::_2)
    );
    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    host_server_ = create_service<waypoint_server_msgs::srv::ServerInfo>(
        "host_server",
        std::bind(&HostServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    update_executor_ = create_publisher<example_interfaces::msg::Empty>("server_update", 10);
}


void waypoint_server::HostServer::ServerApplyAcception(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request,const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response)
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
    handle_tmp.client = create_client<waypoint_server_msgs::srv::ServerInfo>(handle_tmp.server_name, rmw_qos_profile_services_default, client_cb_group_);
    server_handles_.push_back(handle_tmp);

    response->message = "Succes : Server Apply Accepted.";
}


void waypoint_server::HostServer::Callback(const std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Request> request, std::shared_ptr<waypoint_server_msgs::srv::ServerInfo::Response> response)
{
    RCLCPP_INFO(get_logger(), "Accept Request.");

    ServerUpdate();

    server_commands = request->data;
    commands_size = server_commands.size();

    server_result = response;

    result_msg  = "Server Result{ ";
    if(commands_size < 1)
    {
        result_msg += "Non Server Selected ";
        SendResponse();
    }
    else
    {
        command_num = 0;
        ServerCall(server_commands[command_num]);
    }
}

void waypoint_server::HostServer::ServerCall(std::string &command)
{
    auto command_data = Command_Split(command);
    for(ServerHandle &handle : server_handles_)
    {
        if(handle.command_header == command_data[0])
        {
            auto request = std::make_shared<waypoint_server_msgs::srv::ServerInfo::Request>();
            request->data = command_data;
        
            auto future_response = handle.client->async_send_request(
                request, 
                std::bind(&HostServer::RunCommands,this,std::placeholders::_1));
            return;
        }
    }
    result_msg += "Wrong Command->" + command + ", " ;
    command_num++;
    if(command_num < commands_size) ServerCall(server_commands[command_num]);
    else SendResponse();
}

void waypoint_server::HostServer::RunCommands(rclcpp::Client<waypoint_server_msgs::srv::ServerInfo>::SharedFuture future)
{
    result_msg += future.get()->message + ", ";
    command_num++;
    if(command_num < commands_size) ServerCall(server_commands[command_num]);
    else SendResponse();
}

void waypoint_server::HostServer::SendResponse()
{
    result_msg += "}";
    // RCLCPP_INFO(get_logger(), result_msg.c_str());
    server_result->message = result_msg;
    RCLCPP_INFO(get_logger(), server_result->message.c_str());
}


void waypoint_server::HostServer::ServerUpdate()
{
    example_interfaces::msg::Empty msg;
    update_executor_->publish(msg);
}


std::vector<std::string> waypoint_server::HostServer::Command_Split(std::string command_line)
{
    std::vector<std::string> command;
    std::string tmp_str = "";
    for (char c : command_line)
    {
        if (c == ':')
        {
            command.push_back(tmp_str);
            tmp_str = "";
        }
        else 
        {
            tmp_str += c;
        }
    }
    command.push_back(tmp_str);

    return command;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_server::HostServer)