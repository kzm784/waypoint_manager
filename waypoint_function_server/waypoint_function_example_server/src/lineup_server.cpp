#include "waypoint_function_example_server/lineup_server.hpp"

using namespace std::chrono_literals;

waypoint_function::LineupServer::LineupServer(const rclcpp::NodeOptions &options) : FunctionServerNode("lineup_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);

    tarPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("next_waypoint_msg", 10,
        std::bind(&LineupServer::targetPoseCallback, this, std::placeholders::_1));
    curPose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pcl_pose", 10,
        std::bind(&LineupServer::currentPoseCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
        std::bind(&LineupServer::scanCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(0.05s, std::bind(&LineupServer::send_move_direction, this));
    pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
}

void waypoint_function::LineupServer::Update(const std_msgs::msg::Empty::SharedPtr)
{
    lineupAvairable_ = false;
    moveExecute_ = false;
}

void waypoint_function::LineupServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> , std::shared_ptr<waypoint_function_msgs::srv::Command::Response> )
{
    RCLCPP_INFO(get_logger(), "lineup Server Called.");
    lineupAvairable_ = true;
}

void waypoint_function::LineupServer::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tarPose = msg->pose;
}

void waypoint_function::LineupServer::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    curPose = msg->pose.pose;
}

void waypoint_function::LineupServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if(!lineupAvairable_)return;

    float delta_angle = msg->angle_increment;
    int point_counter = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float distance = msg->ranges[i];
      float angle = delta_angle * i + msg->angle_min;
      
      float y = distance * sin(angle);
      if(y*y >= scan_range_y_*scan_range_y_) continue;

      float x = distance * cos(angle);
      if(x >= 0 && x <= scan_range_x_ ) point_counter++;
    }

    // printf("pc:%d, fc:%d\n", point_counter, frame_counter_);
    if(point_counter < scan_tolerance_)
    {
        frame_counter_++;
        if(frame_counter_ > frame_tolerance_)
        {
            moveExecute_ = true;
            // printf("execute to move");
        } 
    }
    else
    {
        moveExecute_ = false;
        frame_counter_ = 0;
    } 
}

void waypoint_function::LineupServer::send_move_direction()
{
    if(!lineupAvairable_)return;

    geometry_msgs::msg::Twist vel_msg{}; 

    if(!moveExecute_)
    {
        pub_vel_->publish(vel_msg);
        return;
    }
    
    float dx = tarPose.position.x - curPose.position.x;
    float dy = tarPose.position.y - curPose.position.y;
    float theta = atan2(dy, dx);
    float dist_sq = dx*dx + dy*dy;

    printf("dist:%f\n", dist_sq);

    if(dist_sq < dist_tolerance_*dist_tolerance_)
    {
        pub_vel_->publish(vel_msg);
        lineupAvairable_ = false;
        moveExecute_ = false;
        finishLineup();
        return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(curPose.orientation, q);
    double roll, pitch, yaw;
    tf2::getEulerYPR(q, yaw, pitch, roll);

    double angle_diff = theta - yaw;
    while (angle_diff > M_PI) angle_diff -= 2*M_PI;
    while (angle_diff < -M_PI) angle_diff += 2*M_PI;
    printf("theta: %f, rot:%f\n", theta, angle_diff);
    vel_msg.angular.z = 0.5 * angle_diff;

    vel_msg.linear.x = 0.2;
    pub_vel_->publish(vel_msg);
}

void waypoint_function::LineupServer::finishLineup()
{
    RCLCPP_INFO(this->get_logger(), "Finish lineup sever");
    std::string result_msg = "line_server:complete";
    SendResponse(result_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::LineupServer)