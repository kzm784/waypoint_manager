#ifndef WAYPOINT_MANAGER__WAYPOINT_MANAGER_UTILS_
#define WAYPOINT_MANAGER__WAYPOINT_MANAGER_UTILS_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace waypoint_manager_utils
{

std::vector<std::vector<std::string>> loadWaypointsFromCSV(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) return {};
    
    std::vector<std::vector<std::string>> waypoints_data;
    std::string line;
    bool is_header = true;

    while (std::getline(file, line))
    {
        std::stringstream line_stream(line);
        std::vector<std::string> row;
        std::string cell;

        if (is_header)
        {
            is_header = false;
            continue;
        }

        while (std::getline(line_stream, cell, ','))
        {
            row.push_back(cell);
        }

        waypoints_data.push_back(row);
    }

    return waypoints_data;
}

}   // namespace waypoint_manager_utils

#endif  // WAYPOINT_MANAGER__WAYPOINT_MANAGER_UTILS_