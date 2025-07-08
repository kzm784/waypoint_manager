#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SKIP_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SKIP_SERVER

#include <waypoint_function_server/function_server_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>

namespace waypoint_function 
{
    class SkipServer : public waypoint_function::FunctionServerNode 
    {
        public:
            explicit SkipServer(const rclcpp::NodeOptions & options);
            void Update(const example_interfaces::msg::Empty::SharedPtr msg) override;
            void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                    std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response) override;

        private:
			void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
			void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
			void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
			void executeSkip();
            float calc_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2);

			bool skipAvairable_;
			float dist_tolerance_;
			int scan_tolerance_;
            geometry_msgs::msg::PoseStamped::SharedPtr tarPose_ptr_;
            geometry_msgs::msg::PoseStamped::SharedPtr curPose_ptr_;

            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tarPose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curPose_sub_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_handle_;

            std::string SERVER_NAME    = "skip_server";
            std::string COMMAND_HEADER = "skip";
            std::string EXECUTE_STATE  = "start";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_SKIP_SERVER