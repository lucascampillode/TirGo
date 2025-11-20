# Install script for directory: /home/tirgopi/carpeta_compartida/catkin_ws_src_backup_20251009-0950/src/biomedical_instrumentation_an_robotics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tirgopi/carpeta_compartida/catkin_ws_src_backup_20251009-0950/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tirgopi/carpeta_compartida/catkin_ws_src_backup_20251009-0950/build/biomedical_instrumentation_an_robotics/catkin_generated/installspace/biomedical_instrumentation_an_robotics.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/biomedical_instrumentation_an_robotics/cmake" TYPE FILE FILES
    "/home/tirgopi/carpeta_compartida/catkin_ws_src_backup_20251009-0950/build/biomedical_instrumentation_an_robotics/catkin_generated/installspace/biomedical_instrumentation_an_roboticsConfig.cmake"
    "/home/tirgopi/carpeta_compartida/catkin_ws_src_backup_20251009-0950/build/biomedical_instrumentation_an_robotics/catkin_generated/installspace/biomedical_instrumentation_an_roboticsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/biomedical_instrumentation_an_robotics" TYPE FILE FILES "/home/tirgopi/carpeta_compartida/catkin_ws_src_backup_20251009-0950/src/biomedical_instrumentation_an_robotics/package.xml")
endif()

