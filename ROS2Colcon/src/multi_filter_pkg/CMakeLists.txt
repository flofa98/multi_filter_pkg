cmake_minimum_required(VERSION 3.8)
project(multi_filter_pkg)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


# Nodes
add_executable(waypoint_nav_node src/waypoint_nav_node.cpp)

ament_target_dependencies(waypoint_nav_node
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(waypoint_nav_node yaml-cpp)

# Installieren
install(TARGETS
  waypoint_nav_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

