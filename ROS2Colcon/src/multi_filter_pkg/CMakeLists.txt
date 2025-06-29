cmake_minimum_required(VERSION 3.5)
project(multi_filter_pkg)

# Finde alle nötigen Pakete
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(multi_filter_node src/multi_filter_node.cpp)

ament_target_dependencies(multi_filter_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

install(TARGETS multi_filter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)



ament_package()

