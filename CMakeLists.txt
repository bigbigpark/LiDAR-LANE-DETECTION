cmake_minimum_required(VERSION 3.0.2)
project(lidar_lane_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  visualization_msgs
  lidar_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp std_msgs visualization_msgs lidar_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lane_detection src/main.cpp src/lane_detection.cpp )
add_dependencies(lane_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_detection ${catkin_LIBRARIES})
