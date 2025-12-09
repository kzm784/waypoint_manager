#ifndef WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_
#define WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <waypoint_function_msgs/srv/command.hpp>
#include <waypoint_manager_utils/waypoint_manager_utils.hpp>

class WaypointNavigator : public rclcpp::Node
{
public:
    explicit WaypointNavigator(const rclcpp::NodeOptions & options);

private:
    enum class ExecuteStage
    {
        Start,
        End
    };

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FunctionCommand = waypoint_function_msgs::srv::Command;

    void onCancelRequested(const std_msgs::msg::String::SharedPtr msg);
    void onFunctionResult(
        const std::shared_ptr<FunctionCommand::Request> request,
        const std::shared_ptr<FunctionCommand::Response> response);

    void publishCurrentWaypoint();
    void publishReachedWaypoint() const;
    void sendNavigationGoal();
    void dispatchFunctionCommands(ExecuteStage stage);
    void handleNavigationResult(
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result);
    void retryCurrentWaypoint();
    void advanceToNextWaypoint();
    bool loadWaypoints(const std::string & csv_path);
    bool handleLapCompletion();
    bool hasActiveWaypoint() const;

    static std::string stageToString(ExecuteStage stage);
    static ExecuteStage stageFromString(const std::string & value);

    // Parameters
    std::string waypoints_csv_;
    bool loop_enable_{false};
    int loop_count_{0};
    int start_id_{0};

    // ROS 2 Interfaces
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr next_waypoint_id_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_waypoint_msg_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reached_waypoint_id_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reached_waypoint_msg_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_pose_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_handle_;
    rclcpp::Client<FunctionCommand>::SharedPtr waypoint_function_client_;
    rclcpp::Service<FunctionCommand>::SharedPtr function_results_receiver_;

    // State variables
    std::vector<waypoint_manager_utils::Waypoint> waypoints_;
    std::size_t waypoint_index_{0};
    bool navigation_finished_{false};
    std::string cancel_state_;
    geometry_msgs::msg::PoseStamped target_pose_;
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_NAVIGATOR_