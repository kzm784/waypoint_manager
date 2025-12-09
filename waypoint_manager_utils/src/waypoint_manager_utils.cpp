#include <waypoint_manager_utils/waypoint_manager_utils.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

std::string trimCopy(const std::string & value)
{
  const auto begin = std::find_if_not(
    value.begin(), value.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
  if (begin == value.end()) {
    return {};
  }

  const auto end = std::find_if_not(
    value.rbegin(), value.rend(), [](unsigned char ch) { return std::isspace(ch) != 0; }).base();

  return std::string(begin, end);
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> cells;
  std::string cell;
  std::istringstream stream(line);

  while (std::getline(stream, cell, ',')) {
    cells.emplace_back(trimCopy(cell));
  }

  if (!line.empty() && line.back() == ',') {
    cells.emplace_back();
  }

  return cells;
}

bool parseWaypoint(const std::vector<std::string> & cells, waypoint_manager_utils::Waypoint & waypoint)
{
  if (cells.size() < 8) {
    return false;
  }

  try {
    waypoint.id = std::stoi(cells[0]);
    waypoint.pose.position.x = std::stod(cells[1]);
    waypoint.pose.position.y = std::stod(cells[2]);
    waypoint.pose.position.z = std::stod(cells[3]);
    waypoint.pose.orientation.x = std::stod(cells[4]);
    waypoint.pose.orientation.y = std::stod(cells[5]);
    waypoint.pose.orientation.z = std::stod(cells[6]);
    waypoint.pose.orientation.w = std::stod(cells[7]);
  } catch (const std::exception &) {
    return false;
  }

  waypoint.commands.clear();
  waypoint.commands.reserve(cells.size() - 8);
  for (size_t index = 8; index < cells.size(); ++index) {
    const auto & command = cells[index];
    if (!command.empty()) {
      waypoint.commands.emplace_back(command);
    }
  }

  return true;
}

}  // namespace

namespace waypoint_manager_utils
{

std::vector<Waypoint> loadWaypointsFromCSV(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "[waypoint_manager_utils] Failed to open CSV: " << file_path << '\n';
    return {};
  }

  std::vector<Waypoint> waypoints;
  std::string line;
  bool skip_header = true;
  std::size_t line_number = 0;

  while (std::getline(file, line)) {
    ++line_number;

    if (skip_header) {
      skip_header = false;
      continue;
    }

    if (trimCopy(line).empty()) {
      continue;
    }

    Waypoint waypoint;
    const auto cells = splitCsvLine(line);
    if (!parseWaypoint(cells, waypoint)) {
      std::cerr << "[waypoint_manager_utils] Skipping invalid row at line " << line_number
                << " in " << file_path << '\n';
      continue;
    }

    waypoints.emplace_back(std::move(waypoint));
  }

  return waypoints;
}

}  // namespace waypoint_manager_utils