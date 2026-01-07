cmake_minimum_required(VERSION 3.0.2)
project(tirgo_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES TirgoMission.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    actionlib_msgs
    message_runtime
)
