#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER

#include <waypoint_function_server_template/function_server.hpp>

#include <unistd.h>

using namespace std;

namespace waypoint_function
{
    class TestServer : public waypoint_function::FunctionServer
    {
        public:
            explicit TestServer(const rclcpp::NodeOptions &options);
            void Update(const example_interfaces::msg::Empty::SharedPtr msg) override;
            void Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                    shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

        private:
            string COMMAND_HEADER = "test";
            string SERVER_NAME = "test_server";
            string EXECUTE_STATE = "start";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER