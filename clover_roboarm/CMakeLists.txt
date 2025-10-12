cmake_minimum_required(VERSION 3.0.2)
project(pca9685_servo_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  SetAngle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs std_srvs message_runtime
)

catkin_install_python(PROGRAMS
  src/servo_controller_node.py
  scripts/pca9685_kill_all.py
  scripts/demo_sweep.py
  scripts/demo_sequence.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)