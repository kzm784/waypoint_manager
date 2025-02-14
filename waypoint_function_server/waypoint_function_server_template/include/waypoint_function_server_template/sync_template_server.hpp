#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__SYNC_TEMPLATE_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__SYNC_TEMPLATE_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

namespace waypoint_function 
{
    class SyncTemplateServer : public rclcpp::Node
    {
        public:
            explicit SyncTemplateServer(const rclcpp::NodeOptions & options);

        private:
            void Update(const example_interfaces::msg::Empty::SharedPtr msg);
            void Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void ServerApply();
        
            rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr update_sub_;
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;

            std::string COMMAND_HEADER = "sync_template";
            std::string SERVER_NAME = "sync_template_server";
            std::string COMMAND_EXAMPLE = "sync_template:10:/path/to/something";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__SYNC_TEMPLATE_SERVER