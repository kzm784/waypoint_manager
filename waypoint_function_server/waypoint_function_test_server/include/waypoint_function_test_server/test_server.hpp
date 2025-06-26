#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER

#include <waypoint_function_server_template/function_server.hpp>

#include <unistd.h>

using namespace std;

class TestServer : public waypoint_function::FunctionServer()
{
    public:
        explicit TestServer(const rclcpp::NodeOptions &options);

    private:
        string COMMAND_HEADER = "test";
        string SERVER_NAME = "test_server";
        string EXECUTE_STATE = "start";
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_TEST_SERVER