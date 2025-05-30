#ifndef WAYPOINT_MANAGER__WAYPOINT_VISUALIZER_
#define WAYPOINT_MANAGER__WAYPOINT_VISUALIZER_

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <waypoint_manager_utils/waypoint_manager_utils.hpp>

#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class WaypointVisualizer : public rclcpp::Node
{
public:
    explicit WaypointVisualizer(const rclcpp::NodeOptions & options);

private:
    void createMarkers();
    void visualizationTimerCallback();
    void nextWaypointIdCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // Parameters
    std::string waypoints_csv_;
    std::string visualization_frame_id_;

    // ROS 2 Interfaces
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reached_waypoint_id_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sphere_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr text_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_markers_pub_;

    // State variables
    int32_t current_waypoint_id_{-1};
    std::vector<std::vector<std::string>> waypoints_data_;
    visualization_msgs::msg::MarkerArray sphere_markers_;
    visualization_msgs::msg::MarkerArray text_markers_;
    visualization_msgs::msg::MarkerArray line_markers_;
    visualization_msgs::msg::MarkerArray arrow_markers_;
};

#endif  // WAYPOINT_MANAGER__WAYPOINT_VISUALIZER_