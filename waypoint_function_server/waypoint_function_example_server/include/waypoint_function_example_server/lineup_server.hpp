#ifndef WAYPOINT_MANAGER__WAYPOINT_FUNCTION_LINEUP_SERVER
#define WAYPOINT_MANAGER__WAYPOINT_FUNCTION_LINEUP_SERVER

#include <waypoint_function_server/function_server_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h> 

#include <cmath>

namespace waypoint_function 
{
    class LineupServer : public waypoint_function::FunctionServerNode 
    {
        public:
            explicit LineupServer(const rclcpp::NodeOptions & options);
            void Update(const std_msgs::msg::Empty::SharedPtr) override;
            void FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> ,
                    std::shared_ptr<waypoint_function_msgs::srv::Command::Response> ) override;

        private:
			void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
			void currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
			void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void send_move_direction();
            void finishLineup();

            bool moveExecute_ = false;
			bool lineupAvairable_ = false;
            int frame_counter_ = 5;
			float dist_tolerance_ = 0.3;
            int frame_tolerance_ = 5;
            float scan_range_x_ = 2.0;
            float scan_range_y_ = 0.5;
			int scan_tolerance_ = 2;

            geometry_msgs::msg::Pose tarPose;
            geometry_msgs::msg::Pose curPose;

            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tarPose_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr curPose_sub_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
            rclcpp::TimerBase::SharedPtr timer_;

            std::string SERVER_NAME    = "lineup_server";
            std::string COMMAND_HEADER = "lineup";
            std::string EXECUTE_STATE  = "start";
    };
}

#endif  // WAYPOINT_MANAGER__WAYPOINT_FUNCTION_LINEUP_SERVER