cmake_minimum_required(VERSION 3.8)
project(tb3_factor_graph_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration)
find_package(GTSAM REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Scan matching wrapper (PCL ICP), built as its own small library so the
# main node file stays focused on graph logic.
add_library(scan_matcher src/scan_matcher.cpp)
target_include_directories(scan_matcher PUBLIC include)
ament_target_dependencies(scan_matcher sensor_msgs)
target_link_libraries(scan_matcher ${PCL_LIBRARIES})

# Log-odds occupancy grid (ray casting from stored scans + keyframe poses).
add_library(occupancy_grid_map src/occupancy_grid_map.cpp)
target_include_directories(occupancy_grid_map PUBLIC include)
ament_target_dependencies(occupancy_grid_map rclcpp nav_msgs sensor_msgs)
target_link_libraries(occupancy_grid_map ${PCL_LIBRARIES})

add_executable(factor_graph_slam_node src/factor_graph_slam_node.cpp)
ament_target_dependencies(factor_graph_slam_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)
target_link_libraries(factor_graph_slam_node
  scan_matcher
  occupancy_grid_map
  ${PCL_LIBRARIES}
  gtsam
)

install(TARGETS
  factor_graph_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()