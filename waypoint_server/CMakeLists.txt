cmake_minimum_required(VERSION 3.8)
project(waypoint_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/host_server.cpp
  src/wait_server.cpp
  src/skip_server.cpp
  src/speak_server.cpp
  src/map_change_server.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "waypoint_server::HostServer"
  EXECUTABLE host_server
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "waypoint_server::WaitServer"
  EXECUTABLE wait_server
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "waypoint_server::SkipServer"
  EXECUTABLE skip_server
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "waypoint_server::SpeakServer"
  EXECUTABLE speak_server
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "waypoint_server::MapChangeServer"
  EXECUTABLE map_change_server
)

target_link_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)

install(FILES DESTINATION share/${PROJECT_NAME})