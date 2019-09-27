cmake_minimum_required(VERSION 2.8.3)
project(roseus_resume)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  control_msgs
  actionlib_msgs
  jsk_gui_msgs
  message_generation
  message_runtime
)

add_message_files(FILES
  ControllerActionStatus.msg
)

add_service_files(FILES
  GetControllerActionStatus.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  control_msgs
  actionlib_msgs
)

catkin_package()
include_directories()
