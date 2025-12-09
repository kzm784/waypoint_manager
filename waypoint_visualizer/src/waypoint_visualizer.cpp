#include "waypoint_visualizer/waypoint_visualizer.hpp"

WaypointVisualizer::WaypointVisualizer(const rclcpp::NodeOptions & options)
: Node("waypoint_visualizer", options)
{
    // Parameters
    declare_parameter<std::string>("waypoints_csv", "");
    declare_parameter<std::string>("visualization_frame_id", "");
    
    get_parameter("waypoints_csv", waypoints_csv_);
    get_parameter("visualization_frame_id", visualization_frame_id_);

    RCLCPP_INFO(get_logger(), "waypoints_csv: %s", waypoints_csv_.c_str());
    RCLCPP_INFO(get_logger(), "visualization_frame_id: %s", visualization_frame_id_.c_str());

    // ROS 2 Interfaces
    reached_waypoint_id_sub_ = create_subscription<std_msgs::msg::Int32>("next_waypoint_id", 10, std::bind(&WaypointVisualizer::nextWaypointIdCallback, this, std::placeholders::_1));;
    sphere_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoint/sphere_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    text_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoint/text_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    line_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoint/line_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    arrow_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoint/arrow_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Load Waypoints from CSV
    waypoints_ = waypoint_manager_utils::loadWaypointsFromCSV(waypoints_csv_);
    if (!waypoints_.empty())
    {
        createMarkers();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "No waypoints loaded. Please check the CSV file.");
        return;
    }
};

void WaypointVisualizer::createMarkers()
{
    sphere_markers_.markers.clear();
    text_markers_.markers.clear();
    line_markers_.markers.clear();
    arrow_markers_.markers.clear();

    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        const auto & waypoint = waypoints_[i];
        const auto & pose = waypoint.pose;

        std::string function_commands;
        for (size_t j = 0; j < waypoint.commands.size(); ++j)
        {
            if (j > 0)
            {
                function_commands += ",";
            }
            function_commands += waypoint.commands[j];
        }

        const double yaw = tf2::getYaw(pose.orientation);

        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header.frame_id = visualization_frame_id_;
        sphere_marker.header.stamp = this->now();
        sphere_marker.ns = "waypoints_sphere";
        sphere_marker.id = static_cast<int>(i);
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;
        sphere_marker.pose.position = pose.position;
        sphere_marker.scale.x = 0.2;
        sphere_marker.scale.y = 0.2;
        sphere_marker.scale.z = 0.2;
        sphere_marker.color.a = 1.0;
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 1.0;
        sphere_marker.color.b = 0.0;
        sphere_markers_.markers.push_back(sphere_marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = visualization_frame_id_;
        text_marker.header.stamp = this->now();
        text_marker.ns = "waypoints_text";
        text_marker.id = static_cast<int>(i);
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.scale.z = 0.3;
        text_marker.color.a = 1.0;
        text_marker.color.r = 0.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0;
        text_marker.text = "ID:" + std::to_string(waypoint.id);
        if (!function_commands.empty())
        {
            text_marker.text += "\nCmd:" + function_commands;
        }

        text_marker.pose.position.x = pose.position.x + 0.3;
        text_marker.pose.position.y = pose.position.y - 0.3;
        text_marker.pose.position.z = pose.position.z + 0.3;

        text_markers_.markers.push_back(text_marker);

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = visualization_frame_id_;
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "waypoints_arrow";
        arrow_marker.id = static_cast<int>(i);
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.scale.x = 0.05;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;
        arrow_marker.color.a = 1.0;
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;

        geometry_msgs::msg::Point start_point;
        start_point.x = pose.position.x;
        start_point.y = pose.position.y;
        start_point.z = pose.position.z;

        geometry_msgs::msg::Point end_point;
        constexpr double arrow_length = 0.3;
        end_point.x = start_point.x + arrow_length * std::cos(yaw);
        end_point.y = start_point.y + arrow_length * std::sin(yaw);
        end_point.z = start_point.z;

        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);

        arrow_markers_.markers.push_back(arrow_marker);

        if (i > 0)
        {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = visualization_frame_id_;
            line_marker.header.stamp = this->now();
            line_marker.ns = "waypoints_line";
            line_marker.id = static_cast<int>(i);
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.02;
            line_marker.color.a = 1.0;
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;

            geometry_msgs::msg::Point prev_point;
            const auto & prev_pose = waypoints_[i - 1].pose;
            prev_point.x = prev_pose.position.x;
            prev_point.y = prev_pose.position.y;
            prev_point.z = prev_pose.position.z;

            geometry_msgs::msg::Point current_point;
            current_point.x = pose.position.x;
            current_point.y = pose.position.y;
            current_point.z = pose.position.z;

            line_marker.points.push_back(prev_point);
            line_marker.points.push_back(current_point);

            line_markers_.markers.push_back(line_marker);
        }
    }

    sphere_markers_pub_->publish(sphere_markers_);
    text_markers_pub_->publish(text_markers_);
    line_markers_pub_->publish(line_markers_);
    arrow_markers_pub_->publish(arrow_markers_);
}

void WaypointVisualizer::nextWaypointIdCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int new_id = msg->data;

    if (current_waypoint_id_ >= 0 && current_waypoint_id_ < (int)sphere_markers_.markers.size()) {
        auto &prev_sphere = sphere_markers_.markers[current_waypoint_id_];
        prev_sphere.color.r = 0.0;
        prev_sphere.color.g = 1.0;
        prev_sphere.color.b = 0.0;

        auto &prev_text = text_markers_.markers[current_waypoint_id_];
        prev_text.color.r = 0.0;
        prev_text.color.g = 0.0;
        prev_text.color.b = 0.0;

        if (current_waypoint_id_ > 0) {
            auto &prev_line = line_markers_.markers[current_waypoint_id_ - 1];
            prev_line.color.r = 0.0;
            prev_line.color.g = 1.0;
            prev_line.color.b = 0.0;
        }
    }

    if (new_id >= 0 && new_id < (int)sphere_markers_.markers.size()) {
        auto &new_sphere = sphere_markers_.markers[new_id];
        new_sphere.color.r = 1.0;
        new_sphere.color.g = 0.0;
        new_sphere.color.b = 0.0;

        auto &new_text = text_markers_.markers[new_id];
        new_text.color.r = 1.0;
        new_text.color.g = 0.0;
        new_text.color.b = 0.0;

        if (new_id > 0 && new_id < (int)line_markers_.markers.size()+1) {
            auto &new_line = line_markers_.markers[new_id - 1];
            new_line.color.r = 1.0;
            new_line.color.g = 0.0;
            new_line.color.b = 0.0;
        }
    }

    sphere_markers_pub_->publish(sphere_markers_);
    text_markers_pub_->publish(text_markers_);
    line_markers_pub_->publish(line_markers_);
    arrow_markers_pub_->publish(arrow_markers_);

    current_waypoint_id_ = new_id;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(WaypointVisualizer)