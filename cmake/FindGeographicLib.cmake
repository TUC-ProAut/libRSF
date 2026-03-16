# Look for GeographicLib
#
# Set
#  GeographicLib_FOUND = GEOGRAPHICLIB_FOUND = TRUE
#  GeographicLib_INCLUDE_DIRS = /usr/local/include
#  GeographicLib_LIBRARIES = /usr/local/lib/libGeographic.so
#  GeographicLib_LIBRARY_DIRS = /usr/local/lib

find_library (GeographicLib_LIBRARIES
  NAMES GeographicLib Geographic
  PATHS "${CMAKE_INSTALL_PREFIX}/../GeographicLib/lib")

find_path (GeographicLib_INCLUDE_DIRS
  NAMES GeographicLib/Config.h
  PATHS
    "${CMAKE_INSTALL_PREFIX}/../GeographicLib/include"
    "${CMAKE_INSTALL_PREFIX}/include")

unset (GeographicLib_LIBRARY_DIRS)
if (GeographicLib_LIBRARIES)
  get_filename_component (GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)
endif ()

unset (GeographicLib_BINARY_DIRS)
if (GeographicLib_INCLUDE_DIRS)
  get_filename_component (_GeographicLib_PREFIX "${GeographicLib_INCLUDE_DIRS}" PATH)
  if (EXISTS "${_GeographicLib_PREFIX}/bin")
    set (GeographicLib_BINARY_DIRS "${_GeographicLib_PREFIX}/bin")
  endif ()
  unset (_GeographicLib_PREFIX)
endif ()

if (NOT GeographicLib_BINARY_DIRS)
  find_program (_GeographicLib_TOOL
    NAMES GeodSolve GeoConvert)
  if (_GeographicLib_TOOL)
    get_filename_component (GeographicLib_BINARY_DIRS "${_GeographicLib_TOOL}" PATH)
  endif ()
  unset (_GeographicLib_TOOL CACHE)
  unset (_GeographicLib_TOOL)
endif ()

# Extract version from GeographicLib/Config.h so that version constraints are enforced
if(GeographicLib_INCLUDE_DIRS)
  file(STRINGS "${GeographicLib_INCLUDE_DIRS}/GeographicLib/Config.h"
    _geo_version_str
    REGEX "^#define GEOGRAPHICLIB_VERSION_STRING \"[^\"]*\"")
  if(_geo_version_str)
    string(REGEX REPLACE ".*\"([^\"]+)\".*" "\\1"
      GeographicLib_VERSION "${_geo_version_str}")
  endif()
  unset(_geo_version_str)
endif()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeographicLib
  REQUIRED_VARS GeographicLib_LIBRARY_DIRS GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS
  VERSION_VAR GeographicLib_VERSION)

if(GeographicLib_FOUND)
  set(GEOGRAPHICLIB_FOUND TRUE)
endif()

if(GeographicLib_FOUND AND NOT TARGET GeographicLib::GeographicLib)
  # Determine the library type from the file extension so that target
  # introspection (get_target_property TYPE) returns the correct value.
  if(GeographicLib_LIBRARIES MATCHES "\\.a$")
    set(_geographiclib_lib_type STATIC)
  else()
    set(_geographiclib_lib_type SHARED)
  endif()
  add_library(GeographicLib::GeographicLib ${_geographiclib_lib_type} IMPORTED)
  set_target_properties(GeographicLib::GeographicLib PROPERTIES
    IMPORTED_LOCATION "${GeographicLib_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIRS}"
  )
  unset(_geographiclib_lib_type)
endif()

mark_as_advanced (GeographicLib_LIBRARY_DIRS GeographicLib_LIBRARIES
  GeographicLib_INCLUDE_DIRS)
