cmake_minimum_required(VERSION 2.8.3)
project(action_interrupt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  control_msgs
  actionlib_msgs
  message_generation
  message_runtime
)

add_message_files(FILES
  InterruptActionStatus.msg
)

add_service_files(FILES
  InterruptController.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  control_msgs
  actionlib_msgs
)

catkin_package()
include_directories()
