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

project(libRSF_library CXX C)

# set source files for library
set(SOURCEFILES
  Data.cpp
  StateData.cpp
  StateDataSet.cpp
  SensorData.cpp
  SensorDataSet.cpp
  FactorGraph.cpp
  FactorGraphConfig.cpp
  QuaternionCalc.cpp
  AngleLocalParametrization.cpp
  NormalizeAngle.cpp
  FileAccess.cpp
  Geometry.cpp
  ListInTime.cpp
  )

# factors of the graph
add_subdirectory(factors)

# predictors to initialize states
add_subdirectory(predictors)

# error models to represent the probabilistic properties
add_subdirectory(error_models)

# set include directory
string(CONCAT INCLUDEDIR ${PROJECT_SOURCE_DIR}/../ "include/")

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

# include all files
include_directories(${INCLUDEDIR})

add_library(libRSF SHARED ${SOURCEFILES} ${HEADERFILES})

target_link_libraries(libRSF ${CERES_LIBRARIES})

