cmake_minimum_required(VERSION 3.0.2)
project(my_truckie)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  control_toolbox
  controller_manager
  joint_state_controller
  robot_state_publisher
  urdf
)

# Add the message files to be generated
add_message_files(
  FILES
  TruckControl.msg
)

# Generate the messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/truck_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



