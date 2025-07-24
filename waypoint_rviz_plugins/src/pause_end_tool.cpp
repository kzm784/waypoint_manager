#include "waypoint_rviz_plugins/pause_end_tool.hpp"

#include <sstream>

#include <OgreVector.h>

#include "rclcpp/qos.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/view_controller.hpp"

namespace waypoint_rviz_plugins
{

PauseEndTool::PauseEndTool() : qos_profile_(1)
{
  shortcut_key_ = 'a';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/pause_end",
    "The topic on which to publish messages.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

void PauseEndTool::onInitialize()
{
  setName("Pause End");
  
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  updateTopic();
}

void PauseEndTool::activate() {
  std_msgs::msg::Bool b;
  b.data = true;
  publisher_->publish(b);
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Sending 'true' to '" <<  topic_property_->getStdString() << "'"
  );
}

void PauseEndTool::deactivate() {}

void PauseEndTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->template create_publisher<std_msgs::msg::Bool>(
    topic_property_->getStdString(), qos_profile_
  );
  clock_ = raw_node->get_clock();
}

void PauseEndTool::updateAutoDeactivate() {}

int PauseEndTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  return Finished;
}

}  // namespace waypoint_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugins::PauseEndTool, rviz_common::Tool)

