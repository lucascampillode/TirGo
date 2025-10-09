########################################################################################################################
#
# CMake version file for the OrocosOCL package.
# This script checks whether the requested version matches exactly or is compatible with the installed package version.
#
# This script sets the following variables:
#  PACKAGE_VERSION: Full provided version string
#  PACKAGE_VERSION_COMPATIBLE: True if version is compatible
#  PACKAGE_VERSION_EXACT: True if version is exact match
#
########################################################################################################################

# Version string
set(PACKAGE_VERSION 2.9.0)

# Exact match test
if(2.9.0 VERSION_EQUAL PACKAGE_FIND_VERSION)
  set(PACKAGE_VERSION_EXACT TRUE)
endif()

# Compatible version test
if(NOT 2.9.0       VERSION_LESS PACKAGE_FIND_VERSION AND    # Installed version can't be lower than requested one
       2 EQUAL        PACKAGE_FIND_VERSION_MAJOR) # ...and major versions must coincide
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
endif()

