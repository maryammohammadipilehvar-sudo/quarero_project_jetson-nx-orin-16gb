# Find livox_sdk2
# This module finds the Livox SDK2 library and headers
#
# This module sets:
#  livox_sdk2_FOUND - True if livox_sdk2 is found
#  livox_sdk2_INCLUDE_DIRS - Include directories for livox_sdk2
#  livox_sdk2_LIBRARIES - Libraries to link against

find_path(livox_sdk2_INCLUDE_DIR
  NAMES livox_lidar_api.h
  PATHS
    /usr/local/include
    /usr/include
    /opt/local/include
  PATH_SUFFIXES
    livox-sdk2
    livox_sdk2
    livox
    sdk_core
    ""
)

find_library(livox_sdk2_LIBRARY
  NAMES
    livox_lidar_sdk_shared
    livox_lidar_sdk
  PATHS
    /usr/local/lib
    /usr/lib
    /opt/local/lib
    /usr/local/lib64
    /usr/lib64
)

# Handle the QUIETLY and REQUIRED arguments and set livox_sdk2_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(livox_sdk2
  FOUND_VAR livox_sdk2_FOUND
  REQUIRED_VARS
    livox_sdk2_LIBRARY
    livox_sdk2_INCLUDE_DIR
)

if(livox_sdk2_FOUND)
  set(livox_sdk2_LIBRARIES ${livox_sdk2_LIBRARY})
  set(livox_sdk2_INCLUDE_DIRS ${livox_sdk2_INCLUDE_DIR})
  
  # Create imported target if it doesn't exist
  if(NOT TARGET livox_sdk2::livox_sdk2)
    add_library(livox_sdk2::livox_sdk2 SHARED IMPORTED)
    set_target_properties(livox_sdk2::livox_sdk2 PROPERTIES
      IMPORTED_LOCATION "${livox_sdk2_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${livox_sdk2_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(
  livox_sdk2_INCLUDE_DIR
  livox_sdk2_LIBRARY
)

