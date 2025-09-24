#include "waypoint_function_example_server/skip_server.hpp"

using namespace std::chrono_literals;

waypoint_function::SkipServer::SkipServer(const rclcpp::NodeOptions &options) : FunctionServerNode("skip_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);

    declare_parameter<float>("dist_tolerance", 3.0);
    declare_parameter<int32_t>("scan_tolerance", 10);
    
    get_parameter("dist_tolerance", dist_tolerance_);
    get_parameter("scan_tolerance", scan_tolerance_);

    tarPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("next_waypoint_msg", 10,
        std::bind(&SkipServer::targetPoseCallback, this, std::placeholders::_1));
    curPose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("current_pose", 10,
        std::bind(&SkipServer::currentPoseCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(),
        std::bind(&SkipServer::scanCallback, this, std::placeholders::_1));

    nav_handle_ = create_publisher<std_msgs::msg::String>("nav2_cancel",10);
}

void waypoint_function::SkipServer::Update(const std_msgs::msg::Empty::SharedPtr)
{
    skipAvairable_ = false;
}

void waypoint_function::SkipServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> /*request*/, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "Skip Server Called.");
    skipAvairable_ = true;
    response->message = "skip_server:activated";
}

void waypoint_function::SkipServer::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tarPose_ = msg->pose;
}

void waypoint_function::SkipServer::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    curPose_ = msg->pose.pose;
}

void waypoint_function::SkipServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr_)
{
    if (!skipAvairable_) return;

    const float tol2 = dist_tolerance_ * dist_tolerance_;
    const float dist2_wp = calc_dist2(tarPose_.position, curPose_.position);
    if (dist2_wp > tol2) return;

    tf2::Quaternion q;
    tf2::fromMsg(curPose_.orientation, q);
    double roll, pitch, yaw;
    tf2::getEulerYPR(q, yaw, pitch, roll);

    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);
    const float rx = curPose_.position.x;
    const float ry = curPose_.position.y;

    int laser_cnt = 0;
    for (size_t i = 0; i < scan_ptr_->ranges.size(); ++i)
    {
        const float r = scan_ptr_->ranges[i];
        if (!std::isfinite(r)) continue;
        if (r < scan_ptr_->range_min || r > scan_ptr_->range_max) continue;

        const float theta_l = scan_ptr_->angle_min + scan_ptr_->angle_increment * static_cast<float>(i);
        const float px_l = r * std::cos(theta_l);
        const float py_l = r * std::sin(theta_l);

        const float px_map = rx + cos_yaw * px_l - sin_yaw * py_l;
        const float py_map = ry + sin_yaw * px_l + cos_yaw * py_l;

        const float dx = tarPose_.position.x - px_map;
        const float dy = tarPose_.position.y - py_map;
        const float d2 = dx*dx + dy*dy;

        if (d2 < tol2)
        {
            ++laser_cnt;
            if (laser_cnt >= scan_tolerance_) break;
        }
    }

    if (laser_cnt >= scan_tolerance_) executeSkip();
}    

void waypoint_function::SkipServer::executeSkip()
{
    skipAvairable_ = false;
    std::cout << "skip_server : Excute waypoint skip" << std::endl;
    std_msgs::msg::String msg;
    msg.data = "Next";
    nav_handle_->publish(msg);
}

float waypoint_function::SkipServer::calc_dist2(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2)
{
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    return dx*dx + dy*dy;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::SkipServer)