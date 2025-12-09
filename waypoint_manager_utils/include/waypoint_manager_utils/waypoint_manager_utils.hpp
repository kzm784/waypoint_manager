#ifndef WAYPOINT_MANAGER__WAYPOINT_MANAGER_UTILS_
#define WAYPOINT_MANAGER__WAYPOINT_MANAGER_UTILS_

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace waypoint_manager_utils
{

struct Waypoint
{
  int32_t id{0};
  geometry_msgs::msg::Pose pose;
  std::vector<std::string> commands;
};

std::vector<Waypoint> loadWaypointsFromCSV(const std::string & file_path);

}  // namespace waypoint_manager_utils

#endif  // WAYPOINT_MANAGER__WAYPOINT_MANAGER_UTILS_