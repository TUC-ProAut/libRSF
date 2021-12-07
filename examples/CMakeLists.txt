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

add_executable(Example_FG_Generic Example_FG_Generic.cpp)
target_link_libraries(Example_FG_Generic libRSF)

add_executable(Example_FG_Range Example_FG_Range.cpp)
target_link_libraries(Example_FG_Range libRSF)

add_executable(Example_FG_Pseudorange Example_FG_Pseudorange.cpp)
target_link_libraries(Example_FG_Pseudorange libRSF)

add_executable(Example_GMM_Estimation_1D Example_GMM_Estimation_1D.cpp)
target_link_libraries(Example_GMM_Estimation_1D libRSF)

add_executable(Example_GMM_Estimation_2D Example_GMM_Estimation_2D.cpp)
target_link_libraries(Example_GMM_Estimation_2D libRSF)

add_executable(Example_Marginalization Example_Marginalization.cpp)
target_link_libraries(Example_Marginalization libRSF)

add_executable(Example_IMU_Single Example_IMU_Single.cpp)
target_link_libraries(Example_IMU_Single libRSF)

add_executable(Example_Odometry Example_Odometry.cpp)
target_link_libraries(Example_Odometry libRSF)
