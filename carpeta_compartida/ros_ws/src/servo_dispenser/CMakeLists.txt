cmake_minimum_required(VERSION 3.0.2)
project(servo_dispenser)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/servo_dispenser_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # pytest (opcional, si integras con catkin)
  # catkin_add_nosetests(test)

  # rostest de flujo
  add_rostest(test/servo_dispenser_flow.test)
endif()
