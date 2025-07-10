#include "waypoint_function_skip_server/skip_server.hpp"

using namespace std::chrono_literals;

waypoint_function::SkipServer::SkipServer(const rclcpp::NodeOptions &options) : FunctionServerNode("skip_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);

    tarPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("next_waypoint_msg", 10,
        std::bind(&SkipServer::targetPoseCallback, this, std::placeholders::_1));
    curPose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("current_pose", 10,
        std::bind(&SkipServer::currentPoseCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
        std::bind(&SkipServer::scanCallback, this, std::placeholders::_1));

    nav_handle_ = create_publisher<std_msgs::msg::String>("nav2_cancel",10);
}

void waypoint_function::SkipServer::Update(const std_msgs::msg::Empty::SharedPtr)
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
    tarPoint = msg->pose.position;
}

void waypoint_function::SkipServer::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    curPoint = msg->pose.pose.position;
}

void waypoint_function::SkipServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr_)
{
    if(!skipAvairable_)return;

    float distance = calc_distance(tarPoint, curPoint);
    if(distance > dist_tolerance_) return;

    int scan_count = 0;
    for(auto value : scan_ptr_->ranges) if(value < dist_tolerance_) scan_count++;
    if(scan_count > scan_tolerance_) executeSkip();
}

void waypoint_function::SkipServer::executeSkip()
{
    skipAvairable_ = false;
    std::cout << "skip_server : Excute waypoint skip" << std::endl;
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