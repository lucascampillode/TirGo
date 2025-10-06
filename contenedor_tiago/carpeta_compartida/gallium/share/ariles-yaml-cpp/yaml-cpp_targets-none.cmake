#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ariles_visitor_yaml_cpp" for configuration "None"
set_property(TARGET ariles_visitor_yaml_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(ariles_visitor_yaml_cpp PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libariles_visitor_yaml_cpp.so"
  IMPORTED_SONAME_NONE "libariles_visitor_yaml_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ariles_visitor_yaml_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_ariles_visitor_yaml_cpp "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libariles_visitor_yaml_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
