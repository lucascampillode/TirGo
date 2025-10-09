set (ariles_VERSION 1.6.7)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was arilesConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

####################################################################################

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/octave_targets.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/octave_targets.cmake")
endif()

set (ariles-octave_LIBRARIES "ariles_visitor_octave")
set (ariles-octave_INCLUDE_DIRS "" "")
set (ariles-octave_LIBRARY_DIRS "")
