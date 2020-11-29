# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
# For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
#
# libRSF is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libRSF is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)

project(libRSF_library VERSION "1.2.0" LANGUAGES CXX)

##################################
# add library files
##################################

# Add main header
list(APPEND HEADERFILES ../include/libRSF.h)

# set source files for library
set(SOURCEFILES
  Data.cpp
  DataSet.cpp
  StateData.cpp
  StateDataSet.cpp
  SensorData.cpp
  SensorDataSet.cpp
  FactorGraph.cpp
  FactorGraphConfig.cpp
  FactorGraphSampling.cpp
  FactorGraphStructure.cpp
  FactorIDSet.cpp
  LocalParametrization.cpp
  NormalizeAngle.cpp
  FileAccess.cpp
  CalculateCovariance.cpp
  Geometry.cpp
  misc.cpp
  Messages.cpp
  Tensor.cpp
  VectorTypes.cpp
  VectorMath.cpp
  Statistics.cpp
  Constants.cpp
  Types.cpp
  GNSS.cpp
  Hash.cpp
  Resampling.cpp
  Marginalization.cpp
  TimeMeasurement.cpp
  NumericalRobust.cpp
  )

# factors of the graph
add_subdirectory(factors)

# error models to represent the probabilistic properties
add_subdirectory(error_models)

# geometric models to represent the relation between states and measurements
add_subdirectory(geometric_models)

# set include directory
string(CONCAT INCLUDEDIR ${PROJECT_SOURCE_DIR}/../include/)

# construct path to header files manually to pass them to the project files of an IDE
list(LENGTH SOURCEFILES LIST_NUMBER_MAX)
set(LIST_NUMBER 0)

WHILE( LIST_NUMBER  LESS LIST_NUMBER_MAX )
  list(GET SOURCEFILES ${LIST_NUMBER} LIST_ELEMENT)
  string(REPLACE "cpp" "h" LIST_ELEMENT ${LIST_ELEMENT})
  string(CONCAT LIST_ELEMENT ${INCLUDEDIR} ${LIST_ELEMENT})
  list(APPEND HEADERFILES ${LIST_ELEMENT})
  MATH( EXPR LIST_NUMBER "${LIST_NUMBER} + 1" )
ENDWHILE( LIST_NUMBER  LESS LIST_NUMBER_MAX )

add_library(libRSF STATIC ${SOURCEFILES} ${HEADERFILES})

# to allow shared libraries to link against static libRSF
set_target_properties(libRSF PROPERTIES POSITION_INDEPENDENT_CODE ON
                                        CMAKE_CXX_STANDARD_REQUIRED ON
                                        CMAKE_CXX_EXTENSIONS OFF
                                        CXX_STANDARD 17)

# set include dir of library depending on current tree (build/install)
target_include_directories(libRSF
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

# set target (libRSF) dependencies
target_link_libraries(libRSF PUBLIC Threads::Threads Eigen3::Eigen Ceres::ceres yaml-cpp ${GeographicLib_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(libRSF PUBLIC OpenMP::OpenMP_CXX)
endif()

# enable all warnings for libRSF (this is just enabled from time to time to check the code quality)
#target_compile_options(libRSF PUBLIC -Wextra -Wpedantic -Wall -fmax-errors=100 -Wno-unused-parameter)

##################################
# install library
##################################

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/libRSF)

install(TARGETS libRSF
    EXPORT librsf-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# copy all header files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include/libRSF)

# export to script
install(EXPORT librsf-targets
  FILE
    libRSFTargets.cmake
  NAMESPACE
    libRSF::
  DESTINATION
    ${INSTALL_CONFIGDIR}
)

# create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/libRSFConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/../cmake/libRSFConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/libRSFConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# install the config, configversion and custom find modules
install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/../cmake/FindGeographicLib.cmake
#    ${CMAKE_CURRENT_LIST_DIR}/../cmake/Findortools.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/libRSFConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/libRSFConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

##################################
# uninstall library
##################################

if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_SOURCE_DIR}/cmake/libRSFUninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/libRSFUninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/libRSFUninstall.cmake)
endif()
