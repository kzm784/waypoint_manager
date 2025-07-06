#ifndef WAYPOINT_RVIZ_PLUGINS__PAUSE_END_TOOL_HPP_
#define WAYPOINT_RVIZ_PLUGINS__PAUSE_END_TOOL_HPP_

#include <OgreVector.h>

#include <QCursor>  // NOLINT cpplint cannot handle the include order here
#include <QObject>  // NOLINT cpplint cannot handle the include order here

#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_common/tool.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
class BoolProperty;
class QosProfileProperty;
}
}

namespace waypoint_rviz_plugins
{

class PauseEndTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PauseEndTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:
  void updateTopic();
  void updateAutoDeactivate();

protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

}  // namespace waypoint_rviz_plugins

#endif  // WAYPOINT_RVIZ_PLUGINS__PAUSE_END_TOOL_HPP_

