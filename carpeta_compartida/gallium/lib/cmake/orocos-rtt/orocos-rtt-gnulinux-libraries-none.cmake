#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "imported_rtt-typekit-gnulinux_plugin" for configuration "None"
set_property(TARGET imported_rtt-typekit-gnulinux_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(imported_rtt-typekit-gnulinux_plugin PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "imported_orocos-rtt-gnulinux_dynamic"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/orocos/gnulinux/types/librtt-typekit-gnulinux.so.2.9.0"
  IMPORTED_SONAME_NONE "librtt-typekit-gnulinux.so.2.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS imported_rtt-typekit-gnulinux_plugin )
list(APPEND _IMPORT_CHECK_FILES_FOR_imported_rtt-typekit-gnulinux_plugin "${_IMPORT_PREFIX}/lib/orocos/gnulinux/types/librtt-typekit-gnulinux.so.2.9.0" )

# Import target "imported_rtt-transport-mqueue-gnulinux_plugin" for configuration "None"
set_property(TARGET imported_rtt-transport-mqueue-gnulinux_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(imported_rtt-transport-mqueue-gnulinux_plugin PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "imported_orocos-rtt-gnulinux_dynamic;imported_orocos-rtt-mqueue-gnulinux_dynamic"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/orocos/gnulinux/types/librtt-transport-mqueue-gnulinux.so.2.9.0"
  IMPORTED_SONAME_NONE "librtt-transport-mqueue-gnulinux.so.2.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS imported_rtt-transport-mqueue-gnulinux_plugin )
list(APPEND _IMPORT_CHECK_FILES_FOR_imported_rtt-transport-mqueue-gnulinux_plugin "${_IMPORT_PREFIX}/lib/orocos/gnulinux/types/librtt-transport-mqueue-gnulinux.so.2.9.0" )

# Import target "imported_orocos-rtt-mqueue-gnulinux_dynamic" for configuration "None"
set_property(TARGET imported_orocos-rtt-mqueue-gnulinux_dynamic APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(imported_orocos-rtt-mqueue-gnulinux_dynamic PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "imported_orocos-rtt-gnulinux_dynamic;/usr/lib/x86_64-linux-gnu/libboost_serialization.so"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/liborocos-rtt-mqueue-gnulinux.so.2.9.0"
  IMPORTED_SONAME_NONE "liborocos-rtt-mqueue-gnulinux.so.2.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS imported_orocos-rtt-mqueue-gnulinux_dynamic )
list(APPEND _IMPORT_CHECK_FILES_FOR_imported_orocos-rtt-mqueue-gnulinux_dynamic "${_IMPORT_PREFIX}/lib/liborocos-rtt-mqueue-gnulinux.so.2.9.0" )

# Import target "imported_orocos-rtt-gnulinux_dynamic" for configuration "None"
set_property(TARGET imported_orocos-rtt-gnulinux_dynamic APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(imported_orocos-rtt-gnulinux_dynamic PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NONE "/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libpthread.so;rt;dl"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/liborocos-rtt-gnulinux.so.2.9.0"
  IMPORTED_SONAME_NONE "liborocos-rtt-gnulinux.so.2.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS imported_orocos-rtt-gnulinux_dynamic )
list(APPEND _IMPORT_CHECK_FILES_FOR_imported_orocos-rtt-gnulinux_dynamic "${_IMPORT_PREFIX}/lib/liborocos-rtt-gnulinux.so.2.9.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
