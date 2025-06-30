cmake_minimum_required(VERSION 3.5)
project(multi_filter_pkg)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pakete finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Executable
add_executable(waypoint_nav_node src/waypoint_nav_node.cpp)

# Abhängigkeiten verknüpfen
ament_target_dependencies(waypoint_nav_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
)

# Installiere ausführbare Datei
install(TARGETS
  waypoint_nav_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Optional: installiere Launch-Dateien, YAML etc.
install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Abschluss
ament_package()
