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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    tarPoint = msg->pose.position;
}

void waypoint_function::SkipServer::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    curPoint = msg->pose.pose.position;
}

void waypoint_function::SkipServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr_)
{
    if (!skipAvairable_) return;

    const float tol2 = dist_tolerance_ * dist_tolerance_;
    const float dist2_wp = calc_dist2(tarPoint, curPoint);
    if (dist2_wp > tol2) return;

    const std::string scan_frame = scan_ptr_->header.frame_id;
    const rclcpp::Time scan_time = scan_ptr_->header.stamp;

    geometry_msgs::msg::PointStamped wp_map, wp_scan;
    wp_map.header.frame_id = "map";
    wp_map.header.stamp    = scan_time;
    wp_map.point           = tarPoint;

    using namespace std::chrono_literals;
    if (!tf_buffer_->canTransform(scan_frame, "map", scan_time, 100ms)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF not available (map->%s)", scan_frame.c_str());
        return;
    }
    try {
        const auto tf_map_to_scan = tf_buffer_->lookupTransform(scan_frame, "map", scan_time, 100ms);
        tf2::doTransform(wp_map, wp_scan, tf_map_to_scan);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF transform failed (map->%s): %s", scan_frame.c_str(), ex.what());
        return;
    }

    const float wx = static_cast<float>(wp_scan.point.x);
    const float wy = static_cast<float>(wp_scan.point.y);

    const float a_min = scan_ptr_->angle_min;
    const float a_inc = scan_ptr_->angle_increment;
    const float r_min = scan_ptr_->range_min;
    const float r_max = scan_ptr_->range_max;

    int scan_count = 0;
    const auto &ranges = scan_ptr_->ranges;
    for (size_t i = 0; i < ranges.size(); ++i) {
        const float r = ranges[i];
        if (!std::isfinite(r)) continue;
        if (r < r_min || r > r_max) continue;

        const float ang = a_min + static_cast<float>(i) * a_inc;
        const float rx  = r * std::cos(ang);
        const float ry  = r * std::sin(ang);

        const float dx = rx - wx;
        const float dy = ry - wy;
        const float d2 = dx*dx + dy*dy;
        if (d2 <= tol2) ++scan_count;
    }

    if (scan_count > scan_tolerance_) {
        executeSkip();
    }
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