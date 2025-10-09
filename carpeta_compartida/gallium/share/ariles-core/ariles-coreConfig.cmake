set (ariles_VERSION 1.6.7)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was arilesConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

####################################################################################

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/yaml-cpp_targets.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/yaml-cpp_targets.cmake")
endif()

set (ariles-core_LIBRARIES "")
set (ariles-core_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include" "/usr/include;/usr/include/eigen3")
set (ariles-core_LIBRARY_DIRS "")
