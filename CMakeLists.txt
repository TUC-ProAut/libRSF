# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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
# configure project
##################################

cmake_minimum_required(VERSION 3.8)

project(libRSF VERSION "2.0.0" LANGUAGES CXX)

# We build as Release by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

##################################
# set build options
##################################

option(LIBRSF_BUILD_TEST "If enabled, the tests get build." OFF)

##################################
# add dependencies
##################################

# path for custom find scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# path for locally installed dependencies
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/externals/install)

# Ceres to solve the NLS problem
find_package(Ceres 2.0 REQUIRED)

# Eigen for vectorized calculations
find_package(Eigen3 3.3.5 REQUIRED NO_MODULE)

# Simple multi-threading
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# yaml cpp to parse config files
find_package(yaml-cpp REQUIRED)

# geographic lib for coordinate frame conversions
find_package(GeographicLib REQUIRED)

# google test
if(LIBRSF_BUILD_TEST)
  # download and install GTest
  include(cmake/InstallGoogleTest.cmake.in)
endif()

##################################
# add project files
##################################

# Add sources of libRSF (this includes the installation routines for the library)
add_subdirectory(src)

# Add simple examples
add_subdirectory(examples)

# Add specific applications
add_subdirectory(applications)

# Add tests
if(LIBRSF_BUILD_TEST)
  enable_testing()
  include(GoogleTest)
  add_subdirectory(test)
endif()
