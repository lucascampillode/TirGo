cmake_minimum_required(VERSION 3.0.2)
project(tirgo_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

## tras find_package(catkin REQUIRED ...):
catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/tirgo_web_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
