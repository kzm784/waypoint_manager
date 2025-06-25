#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__SYNC_TEMPLATE_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__SYNC_TEMPLATE_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

using namespace std;

/*--<Change Name>--------------------------------------*/
/*---Package Name : waypoint_function_server_template--*/
/*---Class Name : SyncTemplateServer-------------------*/
/*---Server Name : sync_template-----------------------*/
/*-----------------------------------------------------*/

namespace waypoint_function 
{
    class SyncTemplateServer : public rclcpp::Node
    {
        public:
            explicit SyncTemplateServer(const rclcpp::NodeOptions & options);

        private:
            void Update(const example_interfaces::msg::Empty::SharedPtr msg);
            void Callback(const shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void ServerApply();
        
            rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr update_sub_;
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;

            string COMMAND_HEADER = "sync_template";
            string SERVER_NAME = "sync_template_server";
            string EXECUTE_STATE = "end";
            string COMMAND_EXAMPLE = "sync_template:10:/path/to/something";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__SYNC_TEMPLATE_SERVER