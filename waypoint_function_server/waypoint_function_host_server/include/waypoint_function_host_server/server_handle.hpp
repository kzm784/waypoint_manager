
namespace waypoint_function 
{
    class ServerHandle
    {
        public:
            std::string command_header;
            std::string server_name;
            std::shared_ptr<rclcpp::Client<waypoint_function_msgs::srv::Command>> client;
    };
}