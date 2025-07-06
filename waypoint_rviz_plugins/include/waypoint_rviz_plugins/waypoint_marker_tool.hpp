#ifndef WAYPOINT_RVIZ_PLUGINGS__WAYPOINT_MARKER_TOOL_HPP_
#define WAYPOINT_RVIZ_PLUGINGS__WAYPOINT_MARKER_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>

namespace waypoint_rviz_plugins
{
class WaypointMarkerTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    WaypointMarkerTool();
    virtual ~WaypointMarkerTool();

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
    visualization_msgs::msg::InteractiveMarker createWaypointMarker(int id, double pose_x, double pose_y);
    void processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & feedback);
    void updateWaypointMarker();

private Q_SLOTS:  

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    std::vector<std::pair<double,double>> waypoints_;
};
    
} // namespace waypoint_rviz_plugins

#endif // WAYPOINT_RVIZ_PLUGINGS__WAYPOINT_MARKER_TOOL_HPP_