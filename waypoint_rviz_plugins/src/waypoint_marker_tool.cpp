#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>

#include "waypoint_rviz_plugins/waypoint_marker_tool.hpp"

namespace waypoint_rviz_plugins
{

WaypointMarkerTool::WaypointMarkerTool() : rviz_common::Tool() {}
WaypointMarkerTool::~WaypointMarkerTool() {}

void WaypointMarkerTool::onInitialize()
{
    setName("Add Waypoint");

    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server",
        nh_,
        rclcpp::SystemDefaultsQoS(),
        rclcpp::SystemDefaultsQoS()
    );
}

int WaypointMarkerTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
    if (event.leftDown())
    {
        auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);
        Ogre::Vector3 intersection = projection.second;
        if (projection.first)
        {
            waypoints_.emplace_back(intersection.x, intersection.y);
            updateWaypointMarker();
            return Finished;
        }
        return Render;
    }
    return Render;
}

visualization_msgs::msg::InteractiveMarker WaypointMarkerTool::createWaypointMarker(int id, double pose_x, double pose_y)
{
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.name = std::to_string(id);
    int_marker.description = "";
    int_marker.scale = 1.0;
    int_marker.pose.position.x = pose_x;
    int_marker.pose.position.y = pose_y;
    int_marker.pose.position.z = 0.0;

    // Position control (sphere)
    visualization_msgs::msg::InteractiveMarkerControl pos_control;
    pos_control.name = "move_position";
    pos_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    {
        visualization_msgs::msg::Marker sphere;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.scale.x = 0.2;
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.2;
        sphere.color.r = 0.0;
        sphere.color.g = 1.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;
        pos_control.markers.push_back(sphere);
    }
    pos_control.always_visible = true;
    pos_control.orientation.w = 0.7071;
    pos_control.orientation.x = 0.0;
    pos_control.orientation.y = 0.7071;
    pos_control.orientation.z = 0.0;
    int_marker.controls.push_back(pos_control);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_default;
    rot_control_default.name = "rotate_yaw_default";
    rot_control_default.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rot_control_default.orientation.w = 0.7071;
    rot_control_default.orientation.x = 0.0;
    rot_control_default.orientation.y = -0.7071;
    rot_control_default.orientation.z = 0.0;
    rot_control_default.always_visible = true;
    int_marker.controls.push_back(rot_control_default);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_arrow;
    rot_control_arrow.name = "rotate_yaw_arrow";
    rot_control_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    rot_control_arrow.orientation.w = 0.7071;
    rot_control_arrow.orientation.x = 0.0;
    rot_control_arrow.orientation.y = -0.7071;
    rot_control_arrow.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker arrow;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = 0.3;
        arrow.scale.y = 0.075;
        arrow.scale.z = 0.075;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        rot_control_arrow.markers.push_back(arrow);
    }
    rot_control_arrow.always_visible = true;
    int_marker.controls.push_back(rot_control_arrow);

    // Text control
    visualization_msgs::msg::InteractiveMarkerControl text_control;
    text_control.name = "display_text";
    text_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    text_control.always_visible = true;
    {
        visualization_msgs::msg::Marker text;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.scale.z = 0.2;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        std::string id_text = "ID:" + std::to_string(id);
        text.text = id_text;
        text.pose.position.x = -0.3;
        text.pose.position.y = -0.3;
        text.pose.position.z = 0.3;
        text_control.markers.push_back(text);
    }
    int_marker.controls.push_back(text_control);

    visualization_msgs::msg::InteractiveMarkerControl menu_control;
    menu_control.name              = "menu";
    menu_control.always_visible    = true;
    menu_control.interaction_mode  = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    int_marker.controls.push_back(menu_control);
    
    visualization_msgs::msg::MenuEntry entry;
    entry.id        = 1;
    entry.parent_id = 0;
    entry.title     = "Delete Waypoint";
    int_marker.menu_entries.push_back(entry);

    return int_marker;
}

void WaypointMarkerTool::processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb)
{
    if (fb->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
        return;
    }
    if (fb->menu_entry_id == 1) {
        int idx = std::stoi(fb->marker_name);
        if (0 <= idx && idx < static_cast<int>(waypoints_.size())) {
            waypoints_.erase(waypoints_.begin() + idx);
            updateWaypointMarker();
            RCLCPP_INFO(nh_->get_logger(), "Deleted waypoint %d", idx);
        }
    }
}

void WaypointMarkerTool::updateWaypointMarker()
{
    server_->clear();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        auto int_marker = createWaypointMarker(static_cast<int>(i), waypoints_[i].first, waypoints_[i].second);
        server_->insert(int_marker, std::bind(&WaypointMarkerTool::processMenuControl, this, std::placeholders::_1));
    }

    server_->applyChanges();
}

void WaypointMarkerTool::activate() {}
void WaypointMarkerTool::deactivate() {}

} // namespace waypoint_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugins::WaypointMarkerTool, rviz_common::Tool)