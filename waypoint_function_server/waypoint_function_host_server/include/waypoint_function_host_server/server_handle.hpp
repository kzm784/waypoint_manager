using namespace std;

namespace waypoint_function 
{
    class ServerHandle
    {
        public:
            string command_header;
            string server_name;
            string execute_state;
            shared_ptr<rclcpp::Client<waypoint_function_msgs::srv::Command>> client;
    };
}