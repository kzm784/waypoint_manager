#include "waypoint_navigator/waypoint_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>

using namespace std::chrono_literals;

WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
: Node("waypoint_navigator", options)
{
    declare_parameter<std::string>("waypoints_csv", "");
    declare_parameter<int>("start_id", 0);
    declare_parameter<bool>("loop_enable", false);
    declare_parameter<int>("loop_count", 0);

    get_parameter("waypoints_csv", waypoints_csv_);
    get_parameter("start_id", start_id_);
    get_parameter("loop_enable", loop_enable_);
    get_parameter("loop_count", loop_count_);

    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "start_id: %d", start_id_);
    RCLCPP_INFO(get_logger(), "loop_enable: %s", loop_enable_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "loop_count: %d", loop_count_);

    next_waypoint_id_pub_ = create_publisher<std_msgs::msg::Int32>("next_waypoint_id", 1);
    next_waypoint_msg_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("next_waypoint_msg", 1);
    reached_waypoint_id_pub_ = create_publisher<std_msgs::msg::Int32>("reached_waypoint_id", 1);
    reached_waypoint_msg_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("reached_waypoint_msg", 1);

    nav2_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    cancel_handle_ = create_subscription<std_msgs::msg::String>(
        "/nav2_cancel", 1,
        std::bind(&WaypointNavigator::onCancelRequested, this, std::placeholders::_1));

    waypoint_function_client_ = create_client<FunctionCommand>("function_commands");
    function_results_receiver_ = create_service<FunctionCommand>(
        "function_results",
        std::bind(&WaypointNavigator::onFunctionResult, this, std::placeholders::_1, std::placeholders::_2));

    if (!loadWaypoints(waypoints_csv_)) {
        RCLCPP_ERROR(get_logger(), "No waypoints loaded. Please check the CSV file.");
        navigation_finished_ = true;
        return;
    }

    if (!hasActiveWaypoint()) {
        RCLCPP_ERROR(
            get_logger(), "Start ID %d is out of range. Available waypoints: %zu.",
            start_id_, waypoints_.size());
        navigation_finished_ = true;
        return;
    }

    publishCurrentWaypoint();
    dispatchFunctionCommands(ExecuteStage::Start);
}

void WaypointNavigator::onCancelRequested(const std_msgs::msg::String::SharedPtr msg)
{
    cancel_state_ = msg ? msg->data : std::string{};
    RCLCPP_INFO(get_logger(), "Cancel navigation requested: %s", cancel_state_.c_str());
    nav2_pose_client_->async_cancel_all_goals();
}

void WaypointNavigator::onFunctionResult(
    const std::shared_ptr<FunctionCommand::Request> request,
    const std::shared_ptr<FunctionCommand::Response> /*response*/)
{
    if (!request) {
        return;
    }

    for (const auto & message : request->data) {
        RCLCPP_INFO(get_logger(), "Function result: %s", message.c_str());
    }

    const auto stage = stageFromString(request->execute_state);
    if (stage == ExecuteStage::Start) {
        sendNavigationGoal();
    } else {
        advanceToNextWaypoint();
    }
}

void WaypointNavigator::publishCurrentWaypoint()
{
    if (!hasActiveWaypoint()) {
        return;
    }

    const auto & waypoint = waypoints_[waypoint_index_];
    target_pose_.header.stamp = get_clock()->now();
    target_pose_.header.frame_id = "map";
    target_pose_.pose = waypoint.pose;

    std_msgs::msg::Int32 next_id;
    next_id.data = static_cast<int32_t>(waypoint_index_);
    next_waypoint_msg_pub_->publish(target_pose_);
    next_waypoint_id_pub_->publish(next_id);
}

void WaypointNavigator::publishReachedWaypoint() const
{
    if (!hasActiveWaypoint()) {
        return;
    }

    std_msgs::msg::Int32 reached_id;
    reached_id.data = static_cast<int32_t>(waypoint_index_);
    reached_waypoint_msg_pub_->publish(target_pose_);
    reached_waypoint_id_pub_->publish(reached_id);
}

void WaypointNavigator::sendNavigationGoal()
{
    if (!hasActiveWaypoint()) {
        return;
    }

    if (!nav2_pose_client_->wait_for_action_server(10s)) {
        RCLCPP_WARN(get_logger(), "Navigation server is not available, waiting...");
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose_;

    RCLCPP_INFO(get_logger(), "Send Waypoint Index: %zu", waypoint_index_);
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = std::bind(&WaypointNavigator::handleNavigationResult, this, std::placeholders::_1);
    nav2_pose_client_->async_send_goal(goal_msg, options);
}

void WaypointNavigator::dispatchFunctionCommands(ExecuteStage stage)
{
    if (navigation_finished_ || !hasActiveWaypoint()) {
        return;
    }

    const auto & commands = waypoints_[waypoint_index_].commands;
    if (commands.empty()) {
        if (stage == ExecuteStage::Start) {
            sendNavigationGoal();
        } else {
            advanceToNextWaypoint();
        }
        return;
    }

    auto request = std::make_shared<FunctionCommand::Request>();
    request->data = commands;
    request->execute_state = stageToString(stage);

    while (!waypoint_function_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(get_logger(), "Interrupted while waiting for function command service.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for function command service...");
    }

    waypoint_function_client_->async_send_request(request);
}

void WaypointNavigator::handleNavigationResult(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Nav2 Result Status: SUCCEEDED");
            dispatchFunctionCommands(ExecuteStage::End);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(get_logger(), "Nav2 Result Status: ABORTED");
            retryCurrentWaypoint();
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Nav2 Result Status: CANCELED");
            if (cancel_state_ == "Next") {
                dispatchFunctionCommands(ExecuteStage::End);
            }
            cancel_state_.clear();
            break;
        default:
            RCLCPP_WARN(get_logger(), "Nav2 Result Status: UNKNOWN");
            break;
    }
}

void WaypointNavigator::retryCurrentWaypoint()
{
    if (!hasActiveWaypoint()) {
        return;
    }

    publishCurrentWaypoint();
    sendNavigationGoal();
}

void WaypointNavigator::advanceToNextWaypoint()
{
    if (!hasActiveWaypoint()) {
        return;
    }

    publishReachedWaypoint();

    ++waypoint_index_;
    if (waypoint_index_ >= waypoints_.size()) {
        if (!handleLapCompletion()) {
            navigation_finished_ = true;
            RCLCPP_INFO(get_logger(), "Completed Navigation!");
            return;
        }
    }

    if (!hasActiveWaypoint()) {
        return;
    }

    publishCurrentWaypoint();
    dispatchFunctionCommands(ExecuteStage::Start);
}

bool WaypointNavigator::loadWaypoints(const std::string & csv_path)
{
    waypoints_ = waypoint_manager_utils::loadWaypointsFromCSV(csv_path);
    if (waypoints_.empty()) {
        return false;
    }

    if (start_id_ < 0) {
        RCLCPP_WARN(get_logger(), "start_id %d is negative, defaulting to 0.", start_id_);
        start_id_ = 0;
    }

    if (static_cast<std::size_t>(start_id_) >= waypoints_.size()) {
        return false;
    }

    waypoint_index_ = static_cast<std::size_t>(start_id_);
    return true;
}

bool WaypointNavigator::handleLapCompletion()
{
    if (!loop_enable_) {
        return false;
    }

    if (loop_count_ <= 0) {
        return false;
    }

    --loop_count_;
    if (loop_count_ < 1) {
        return false;
    }

    RCLCPP_INFO(get_logger(), "Completed a lap, %d laps left.", loop_count_);
    waypoint_index_ = 0;
    return true;
}

bool WaypointNavigator::hasActiveWaypoint() const
{
    return !navigation_finished_ && waypoint_index_ < waypoints_.size();
}

std::string WaypointNavigator::stageToString(ExecuteStage stage)
{
    return stage == ExecuteStage::Start ? "start" : "end";
}

WaypointNavigator::ExecuteStage WaypointNavigator::stageFromString(const std::string & value)
{
    std::string normalized(value);
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return normalized == "start" ? ExecuteStage::Start : ExecuteStage::End;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(WaypointNavigator)