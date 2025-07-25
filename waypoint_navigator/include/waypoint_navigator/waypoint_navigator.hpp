#ifndef WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_
#define WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <waypoint_manager_utils/waypoint_manager_utils.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <waypoint_function_msgs/srv/command.hpp>

using namespace std;

class WaypointNavigator : public rclcpp::Node
{
public:
    explicit WaypointNavigator(const rclcpp::NodeOptions & options);

private:
    void UpdateWaypointID();
    void UpdateGoal();
    void SendGoal();
    void UpdateCommands();
    void SendCommands(string execute_state);
    void ReceiveFunctionResults(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,const std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
    void CancelHandle(const std_msgs::msg::String::SharedPtr msg);
    void ToNextWaypoint();
    void ToSameWaypoint();

    // Parameters
    std::string waypoints_csv_;
    bool loop_enable_{false};
    int loop_count_{0};
    int waypoint_id_{0};

    // ROS 2 Interfaces
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr next_waypoint_id_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_waypoint_msg_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reached_waypoint_id_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reached_waypoint_msg_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_pose_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_handle_;
    rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr waypoint_function_client_;
    rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr function_results_reciever_;

    // State variables
    bool retry_once_{false};
    bool skip_enable_{false};
    bool function_enable_{false};
    bool nav2_enable_{true};
    std::string cancelState_ ;
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    std::vector<std::vector<std::string>> waypoints_data_;
    std::vector<std::string> function_commands_;
    geometry_msgs::msg::PoseStamped target_pose_;
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_