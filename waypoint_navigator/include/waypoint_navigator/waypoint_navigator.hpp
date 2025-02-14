#ifndef WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_
#define WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <waypoint_manager_utils/waypoint_manager_utils.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <example_interfaces/msg/string.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

class WaypointNavigator : public rclcpp::Node
{
public:
    explicit WaypointNavigator(const rclcpp::NodeOptions & options);

private:
    void sendGoal();
    void updateWaypoint();
    void ReceiveFunctionResults(const example_interfaces::msg::String::SharedPtr msg);
    void updateGoal();
    void cancleHandle(const example_interfaces::msg::Empty::SharedPtr msg);

    // Parameters
    std::string waypoints_csv_;
    bool loop_enable_{false};
    int loop_count_{0};
    int waypoint_id_{0};

    // ROS 2 Interfaces
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr next_waypoint_id_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reached_waypoint_id_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_pose_client_;
    rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr cancle_nav_handle_;
    rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr waypoint_function_client_;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr function_result_sub_;

    // State variables
    bool retry_once_{false};
    bool skip_enable_{false};
    bool function_enable_{false};
    bool nav2_enable_{true};
    std::vector<std::vector<std::string>> waypoints_data_;
    geometry_msgs::msg::PoseStamped target_pose;
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_