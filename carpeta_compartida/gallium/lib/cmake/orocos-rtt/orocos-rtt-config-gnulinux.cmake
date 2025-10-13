########################################################################################################################
#  Target-specific definitions. This is currently only the include paths.
#
########################################################################################################################


# Prefix used to distinguish imported targets from regular ones
set(PREFIX imported_)

# Path to headers
get_filename_component(OROCOS-RTT_INCLUDE_DIRS "${SELF_DIR}/../../../include" ABSOLUTE)
list( APPEND OROCOS-RTT_INCLUDE_DIRS ${OROCOS-RTT_INCLUDE_DIRS}/orocos "/usr/include;/usr/include"   )

#Used CORBA IMPLEMENTATION
set(CORBA_IMPLEMENTATION TAO)
