#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>
#include <fstream>

#include "waypoint_rviz_plugins/waypoint_marker_tool.hpp"

using namespace std::placeholders;

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
    line_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("waypoint_line", 10);
    line_timer_ = nh_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&WaypointMarkerTool::publishLineMarker, this)
    );
    save_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "save_waypoints",
        std::bind(&WaypointMarkerTool::handleSaveWaypoints, this, _1, _2)
    );
    load_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "load_waypoints",
        std::bind(&WaypointMarkerTool::handleLoadWaypoints, this, _1, _2)
    );

    waypoints_.clear();
}

int WaypointMarkerTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
    if (event.leftDown())
    {
        auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);
        Ogre::Vector3 intersection = projection.second;
        if (projection.first)
        {
            Waypoint wp;
            wp.pose.header.frame_id = "map";
            wp.pose.header.stamp = nh_->now();
            wp.pose.pose.position.x = projection.second.x;
            wp.pose.pose.position.y = projection.second.y;
            wp.pose.pose.position.z = 0.0;
            wp.pose.pose.orientation.x = 0.0;
            wp.pose.pose.orientation.y = 0.0;
            wp.pose.pose.orientation.z = 0.0;
            wp.pose.pose.orientation.w = 1.0;
            wp.function_command.clear();
            waypoints_.push_back(std::move(wp));
            updateWaypointMarker();
            return Finished;
        }
        return Render;
    }
    return Render;
}

void WaypointMarkerTool::updateWaypointMarker()
{
    server_->clear();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        auto int_marker = createWaypointMarker(static_cast<int>(i));
        server_->insert(int_marker, std::bind(&WaypointMarkerTool::processFeedback, this, _1));
    }

    server_->applyChanges();
}

visualization_msgs::msg::InteractiveMarker WaypointMarkerTool::createWaypointMarker(const int id)
{
    const auto & wp = waypoints_[id];

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = wp.pose.header.frame_id;;
    int_marker.name = std::to_string(id);
    int_marker.description = waypoints_.at(id).function_command;
    int_marker.scale = 1.0;
    int_marker.pose.position = wp.pose.pose.position;
    int_marker.pose.orientation = wp.pose.pose.orientation;

    // Position control (sphere)
    visualization_msgs::msg::InteractiveMarkerControl pos_control;
    pos_control.name = "move_position";
    pos_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    pos_control.always_visible = true;
    pos_control.orientation.w = 0.7071;
    pos_control.orientation.x = 0.0;
    pos_control.orientation.y = 0.7071;
    pos_control.orientation.z = 0.0;
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
        std::string id_text = "ID:" + std::to_string(id) + "\n" + waypoints_.at(id).function_command;;
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
    
    visualization_msgs::msg::MenuEntry delete_entry;
    delete_entry.id        = 1;
    delete_entry.parent_id = 0;
    delete_entry.title     = "Delete Waypoint";
    int_marker.menu_entries.push_back(delete_entry);

    visualization_msgs::msg::MenuEntry add_function_command_entry;
    add_function_command_entry.id = 2;
    add_function_command_entry.parent_id = 0;
    add_function_command_entry.title = "Add Function Command";
    int_marker.menu_entries.push_back(add_function_command_entry);

    return int_marker;
}

void WaypointMarkerTool::processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb)
{
    int id = std::stoi(fb->marker_name);

    switch (fb->event_type)
    {
        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
            waypoints_[id].pose.pose.position = fb->pose.position;
            waypoints_[id].pose.pose.orientation = fb->pose.orientation;
            updateWaypointMarker();
            break;

        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
            processMenuControl(fb);
            break;

        default:
            break;
    }    
}

void WaypointMarkerTool::processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb)
{
    if (fb->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) { return; }

    int idx = std::stoi(fb->marker_name);
    if (idx < 0 || idx >= static_cast<int>(waypoints_.size())) { return; }

    switch (fb->menu_entry_id) {
      
        // Delete Waypoint
        case 1:
            waypoints_.erase(waypoints_.begin() + idx);
            updateWaypointMarker();
            RCLCPP_INFO(nh_->get_logger(), "Deleted waypoint %d", idx);
            break;

        // Add / Edit Function Command
        case 2:
        {
            bool ok = false;
            QString current = QString::fromStdString(waypoints_[idx].function_command);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Edit Function Command"),
                tr("Enter command for waypoint %1:").arg(idx),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) 
            {
                waypoints_[idx].function_command = text.toStdString();
                updateWaypointMarker();
                RCLCPP_INFO(nh_->get_logger(), "Updated command of waypoint %d to '%s'", idx, waypoints_[idx].function_command.c_str());
            }
        }
        break;

      default:
        break;
    }
}

void WaypointMarkerTool::publishLineMarker()
{
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp    = nh_->now();
    line.ns              = "waypoint_lines";
    line.id              = 0;
    line.type            = visualization_msgs::msg::Marker::LINE_LIST;
    line.action          = visualization_msgs::msg::Marker::ADD;
    line.scale.x         = 0.025f;
    line.color.r         = 0.0f;
    line.color.g         = 1.0f;
    line.color.b         = 0.0f;
    line.color.a         = 1.0f;

    for (size_t i = 1; i < waypoints_.size(); ++i) {
        geometry_msgs::msg::Point p0 = waypoints_[i-1].pose.pose.position;
        geometry_msgs::msg::Point p1 = waypoints_[i].pose.pose.position;
        line.points.push_back(p0);
        line.points.push_back(p1);
    }

    line_pub_->publish(line);
}

void WaypointMarkerTool::handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    QString qpath = QFileDialog::getSaveFileName(
        nullptr,
        tr("Save Waypoints As"),
        "",
        tr("CSV Files (*.csv)")
    );
    if (qpath.isEmpty()) {
        res->success = false;
        res->message = "Save canceled by user";
        return;
    }

    if (!qpath.endsWith(".csv", Qt::CaseInsensitive)) {
        qpath += ".csv";
    }

    const std::string path = qpath.toStdString();

    std::ofstream ofs(path);
    if (!ofs) {
        QMessageBox::warning(nullptr, tr("Error"),
        tr("Cannot open file:\n%1").arg(qpath));
        res->success = false;
        res->message = "Failed to open file: " + path;
        return;
    }

    ofs << "id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command\n";
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const auto & p = waypoints_[i].pose.pose;
        ofs
        << i << ","
        << p.position.x << "," << p.position.y << "," << p.position.z << ","
        << p.orientation.x << "," << p.orientation.y << ","
        << p.orientation.z << "," << p.orientation.w << ","
        << "\"" << waypoints_[i].function_command << "\"\n";
    }
    ofs.close();

    res->success = true;
    res->message = "Saved " + std::to_string(waypoints_.size()) + " waypoints to " + path;
}

void WaypointMarkerTool::handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    QString qpath = QFileDialog::getOpenFileName(
        nullptr,
        tr("Open Waypoints"),
        "",
        tr("CSV Files (*.csv)"));
    if (qpath.isEmpty()) {
        res->success = false;
        res->message = "Load canceled by user";
    return;
    }

    QFile file(qpath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(nullptr, tr("Error"),
        tr("Cannot open file:\n%1").arg(qpath));
    res->success = false;
    res->message = "Failed to open file: " + qpath.toStdString();
    return;
    }

    QTextStream in(&file);
    QString header = in.readLine();

    waypoints_.clear();

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        QStringList cols = line.split(',');
        if (cols.size() < 9) continue;

        Waypoint wp;
        wp.pose.header.frame_id = "map";
        wp.pose.header.stamp = nh_->now();
        wp.pose.pose.position.x = cols[1].toDouble();
        wp.pose.pose.position.y = cols[2].toDouble();
        wp.pose.pose.position.z = cols[3].toDouble();
        wp.pose.pose.orientation.x = cols[4].toDouble();
        wp.pose.pose.orientation.y = cols[5].toDouble();
        wp.pose.pose.orientation.z = cols[6].toDouble();
        wp.pose.pose.orientation.w = cols[7].toDouble();
        QString cmd;
        if (cols.size() >= 9) {
            QStringList cmdParts = cols.mid(8);
            cmd = cmdParts.join(",");
            if (cmd.startsWith('"') && cmd.endsWith('"') && cmd.size() >= 2) {
                cmd = cmd.mid(1, cmd.size() - 2);
            }
        }
        wp.function_command = cmd.toStdString();

        waypoints_.push_back(std::move(wp));
    }
    file.close();

    updateWaypointMarker();

    res->success = true;
    res->message = "Loaded " + std::to_string(waypoints_.size()) + " waypoints from " + qpath.toStdString();
}

void WaypointMarkerTool::activate() {}
void WaypointMarkerTool::deactivate() {}

} // namespace waypoint_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugins::WaypointMarkerTool, rviz_common::Tool)