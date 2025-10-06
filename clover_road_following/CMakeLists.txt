cmake_minimum_required(VERSION 3.0.2)
project(clover_road_following)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python nodes are in scripts/; ensure install if needed (optional)