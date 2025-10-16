cmake_minimum_required(VERSION 3.0.2)
project(stt_vosk)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

## Si tu paquete NO tiene headers C/C++, basta con incluir los de catkin
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Instala los scripts Python (asegúrate de que existen y son ejecutables)
catkin_install_python(PROGRAMS
  scripts/stt_vosk_node.py
  scripts/list_audio_devices.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# (Opcional) Instala los launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
