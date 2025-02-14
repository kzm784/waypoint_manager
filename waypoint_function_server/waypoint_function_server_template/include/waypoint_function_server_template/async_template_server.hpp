#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__ASYNC_TEMPLATE_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__ASYNC_TEMPLATE_SERVER

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

namespace waypoint_function 
{
    class AsyncTemplateServer : public rclcpp::Node
    {
        public:
            explicit AsyncTemplateServer(const rclcpp::NodeOptions & options);

        private:
            void Update(const example_interfaces::msg::Empty::SharedPtr msg);
            void Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void TimerCallback();
            void SendAsyncResponse();
            void ServerApply();
        
            rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr update_sub_;
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr response_pub_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::string COMMAND_HEADER = "async_template";
            std::string SERVER_NAME = "async_template_server";
            std::string COMMAND_EXAMPLE = "async_template:";

            bool _isAvalable;
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SERVER_TEMPLATE__ASYNC_TEMPLATE_SERVER