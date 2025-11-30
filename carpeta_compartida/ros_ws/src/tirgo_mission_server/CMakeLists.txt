cmake_minimum_required(VERSION 3.0.2)
project(tirgo_mission_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  tirgo_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/tirgo_mission_server.py
    scripts/test_tirgo_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(test/test_mission_flow.test)
endif()
