#include "waypoint_function_skip_server/skip_server.hpp"

using namespace std::chrono_literals;

waypoint_function::SkipServer::SkipServer(const rclcpp::NodeOptions &options) : FunctionServerNode("skip_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);

    tarPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("next_waypoint_msg", 1,
        std::bind(&SkipServer::targetPoseCallback, this, std::placeholders::_1));
    curPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("pcl_pose", 1,
        std::bind(&SkipServer::currentPoseCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("sacn", 1,
        std::bind(&SkipServer::scanCallback, this, std::placeholders::_1));

    nav_handle_ = create_publisher<std_msgs::msg::String>("navigation_handle",10);
}

void waypoint_function::SkipServer::Update(const example_interfaces::msg::Empty::SharedPtr msg)
{
    skipAvairable_ = false;
}

void waypoint_function::SkipServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Skip Server Called.");
    skipAvairable_ = true;
    response->message = "skip_server:activated";
}

void waypoint_function::SkipServer::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tarPose_ptr_ = msg;
}

void waypoint_function::SkipServer::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    curPose_ptr_ = msg;
}

void waypoint_function::SkipServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr_)
{
    if(!skipAvairable_)return;
    float distance = calc_distance(tarPose_ptr_->pose.position, curPose_ptr_->pose.position);
    if(distance > dist_tolerance_) return;

    int scan_count = 0;
    for(auto value : scan_ptr_->ranges)
    {
        if(value < dist_tolerance_) scan_count++;
    }
    if(scan_count > scan_tolerance_) executeSkip();
}

void waypoint_function::SkipServer::executeSkip()
{
    std_msgs::msg::String msg;
    msg.data = "Next";
    nav_handle_->publish(msg);
}

float waypoint_function::SkipServer::calc_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2)
{
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    return std::sqrt(dx*dx + dy*dy);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::SkipServer)