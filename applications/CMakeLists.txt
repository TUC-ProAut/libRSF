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

add_executable(App_multiFusion App_multiFusion.cpp AppPool_Sensors.cpp AppPool_Init.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp)
target_link_libraries(App_multiFusion libRSF)

add_executable(App_GNSS App_GNSS.cpp AppPool_Sensors.cpp AppPool_Init.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp)
target_link_libraries(App_GNSS libRSF)

add_executable(App_SLAM App_SLAM.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp)
target_link_libraries(App_SLAM libRSF)

add_executable(App_SLAM_BR_2D App_SLAM_BR_2D.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp)
target_link_libraries(App_SLAM_BR_2D libRSF)

add_executable(App_Ranging_2D App_Ranging_2D.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp AppPool_Sensors.cpp)
target_link_libraries(App_Ranging_2D libRSF)

add_executable(App_SLAM_RO_2D App_SLAM_RO_2D.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp AppPool_Sensors.cpp)
target_link_libraries(App_SLAM_RO_2D libRSF)

add_executable(App_SLAM_Range_Loop_2D App_SLAM_Range_Loop_2D.cpp AppPool_Adaptive.cpp AppPool_Defines.cpp AppPool_Utility.cpp AppPool_Sensors.cpp)
target_link_libraries(App_SLAM_Range_Loop_2D libRSF)

add_executable(App_PointSetRegistration App_PointSetRegistration.cpp AppPool_Defines.cpp)
target_link_libraries(App_PointSetRegistration libRSF)

add_executable(App_PointSetRegistration_Simple App_PointSetRegistration_Simple.cpp AppPool_Defines.cpp)
target_link_libraries(App_PointSetRegistration_Simple libRSF)

add_executable(App_Robust_Models_1D App_Robust_Models_1D.cpp)
target_link_libraries(App_Robust_Models_1D libRSF)

add_executable(App_Robust_Models_2D App_Robust_Models_2D.cpp)
target_link_libraries(App_Robust_Models_2D libRSF)

add_executable(App_Robust_Models_3D App_Robust_Models_3D.cpp)
target_link_libraries(App_Robust_Models_3D libRSF)

add_executable(ICRA19_Ranging ICRA19_Ranging.cpp)
target_link_libraries(ICRA19_Ranging libRSF)

add_executable(ICRA19_GNSS ICRA19_GNSS.cpp)
target_link_libraries(ICRA19_GNSS libRSF)

add_executable(IV19_GNSS IV19_GNSS.cpp)
target_link_libraries(IV19_GNSS libRSF)
