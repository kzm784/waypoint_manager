#include "waypoint_navigator/waypoint_navigator.hpp"

using namespace waypoint_manager_utils;
using namespace std::chrono_literals;

WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions & options)
: Node("waypoint_navigator", options)
{
    // Parameters
    declare_parameter<std::string>("waypoints_csv", "");
    declare_parameter<std::int32_t>("start_waypoint_id", 0);
    declare_parameter<bool>("loop_enable", false);
    declare_parameter<std::int32_t>("loop_count", 0);

    get_parameter("waypoints_csv", waypoints_csv_);
    get_parameter("start_waypoint_id", waypoint_id_);
    get_parameter("loop_enable", loop_enable_);
    get_parameter("loop_count", loop_count_);

    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "start_waypoint_id: %d", waypoint_id_);
    RCLCPP_INFO(get_logger(), "loop_enable: %s", loop_enable_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "loop_count: %d", loop_count_);

    // Publishers
    next_waypoint_id_pub_ = create_publisher<std_msgs::msg::Int32>("next_waypoint_id", 1);
    reached_waypoint_id_pub_ = create_publisher<std_msgs::msg::Int32>("reached_waypoint_id", 1);

    // Action client for navigation
    nav2_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    // Nav2 action handle
    cancle_nav_handle_ = create_subscription<example_interfaces::msg::Empty>("/cancle_nav", 1,
                            bind(&WaypointNavigator::cancleHandle, this, std::placeholders::_1));

    // Client to request waypoint_event
    waypoint_event_client_ = create_client<waypoint_event_msgs::srv::Command>("waypoint_event/event_commands");
    // Sibscriber to recieve wapoint_server results
    event_result_sub_ = create_subscription<example_interfaces::msg::String>("waypoint_event/event_result", 1,
        bind(&WaypointNavigator::ReceiveEventResults, this, std::placeholders::_1));

    // Load Waypoints from CSV
    waypoints_data_ = loadWaypointsFromCSV(waypoints_csv_);
    if (waypoints_data_.empty())
    {
        RCLCPP_ERROR(get_logger(), "No waypoints loaded. Please check the CSV file.");
        return;
    }

    // Start waypoint navigation
    updateGoal();
}

void WaypointNavigator::updateGoal()
{
    if (waypoints_data_.empty() || waypoint_id_ >= static_cast<int>(waypoints_data_.size())) return;

    // RCLCPP_INFO(get_logger(), "Update waypoint ID: %d", waypoint_id_);

    // Create PoseStamped for goal
    target_pose.header.stamp = get_clock()->now();
    target_pose.header.frame_id = "map";
    target_pose.pose.position.x = std::stof(waypoints_data_[waypoint_id_][1]);
    target_pose.pose.position.y = std::stof(waypoints_data_[waypoint_id_][2]);
    target_pose.pose.position.z = std::stof(waypoints_data_[waypoint_id_][3]);
    target_pose.pose.orientation.x = std::stof(waypoints_data_[waypoint_id_][4]);
    target_pose.pose.orientation.y = std::stof(waypoints_data_[waypoint_id_][5]);
    target_pose.pose.orientation.z = std::stof(waypoints_data_[waypoint_id_][6]);
    target_pose.pose.orientation.w = std::stof(waypoints_data_[waypoint_id_][7]);

    // Publish next waypoint ID
    std_msgs::msg::Int32 next_waypoint_msg;
    next_waypoint_msg.data = waypoint_id_;
    next_waypoint_id_pub_->publish(next_waypoint_msg);
    
    auto request = std::make_shared<waypoint_event_msgs::srv::Command::Request>();

    int data_length = waypoints_data_[waypoint_id_].size();
    for (int i=8; i<data_length; i++)
    {
        request->data.push_back(waypoints_data_[waypoint_id_][i]);
    }

    while (!waypoint_event_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
      }
    waypoint_event_client_->async_send_request(request);
        
}

void WaypointNavigator::ReceiveEventResults(example_interfaces::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), msg->data.c_str());
    sendGoal();
}

void WaypointNavigator::updateWaypoint()
{
    // Publish reached waypoint ID
    std_msgs::msg::Int32 reached_waypoint_msg;
    reached_waypoint_msg.data = waypoint_id_;
    reached_waypoint_id_pub_->publish(reached_waypoint_msg);

    waypoint_id_++;
    if (waypoint_id_ >= static_cast<int>(waypoints_data_.size()))
    {
        loop_count_--;
        if (!loop_enable_ || (loop_count_ < 1))
        {
            RCLCPP_INFO(get_logger(), "Completed Navigation!");
            return;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Completed a lap, %d laps left.", loop_count_);
            waypoint_id_ = 0;
            return;
        }
    }
}

void WaypointNavigator::sendGoal()
{
    if (!nav2_pose_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_WARN(get_logger(), "Navigation server is not available, waiting...");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = target_pose;

    RCLCPP_INFO(get_logger(), "Send Waypoint ID: %d", waypoint_id_ );
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const auto& result) {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                updateWaypoint();
                updateGoal();
                break;

            case rclcpp_action::ResultCode::ABORTED:
                break;

            case rclcpp_action::ResultCode::CANCELED:
                break;

            default:
                break;
            }
        };

    nav2_pose_client_->async_send_goal(goal_msg, send_goal_options);
}


void WaypointNavigator::cancleHandle(const example_interfaces::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Nav2 Cancle Called");
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(WaypointNavigator)