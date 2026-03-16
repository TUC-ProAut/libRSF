# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
# Copyright (C) 2026 Tim Pfeifer
# For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
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

##################################
# add library files
##################################

# set source files for library
set(SOURCEFILES
  Data.cpp
  DataGeneric.cpp
  DataConfig.cpp
  DataStream.cpp
  DataSet.cpp
  StateDataSet.cpp
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
  Misc.cpp
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

# collect header files for IDE visibility
file(GLOB_RECURSE HEADERFILES "${CMAKE_SOURCE_DIR}/include/*.h")

add_library(libRSF STATIC ${SOURCEFILES} ${HEADERFILES})
add_library(libRSF::libRSF ALIAS libRSF)

# factors of the graph
add_subdirectory(factors)

# error models to represent the probabilistic properties
add_subdirectory(error_models)

# geometric models to represent the relation between states and measurements
add_subdirectory(geometric_models)

# require at least C++ 17
target_compile_features(libRSF PUBLIC cxx_std_17)

set(LIBRSF_YAML_TARGET yaml-cpp::yaml-cpp)
if(NOT TARGET yaml-cpp::yaml-cpp)
  set(LIBRSF_YAML_TARGET yaml-cpp)
endif()

set_target_properties(libRSF PROPERTIES CXX_EXTENSIONS OFF)

# set include dir of library depending on current tree (build/install)
target_include_directories(libRSF
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

# set target (libRSF) dependencies
target_link_libraries(libRSF PUBLIC Threads::Threads Eigen3::Eigen Ceres::ceres ${LIBRSF_YAML_TARGET} GeographicLib::GeographicLib)

# optional extra warnings — enable with -DLIBRSF_EXTRA_WARNINGS=ON
option(LIBRSF_EXTRA_WARNINGS "Enable extra compiler warnings for libRSF" OFF)
if(LIBRSF_EXTRA_WARNINGS)
  target_compile_options(libRSF PRIVATE -Wextra -Wpedantic -Wall -fmax-errors=100 -Wno-unused-parameter)
endif()

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
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

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

# install the Config, ConfigVersion and custom find modules
install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/../cmake/FindGeographicLib.cmake
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
