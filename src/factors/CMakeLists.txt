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

#set source files for library
  set(SOURCEFILES ${SOURCEFILES}
  factors/BaseFactor.cpp
  factors/ConstantValueFactor.cpp
  factors/ConstantDriftFactor.cpp
  factors/ConstantQuaternionFactor.cpp
  factors/ConstantPose2Factor.cpp
  factors/BetweenValueFactor.cpp
  factors/BetweenPose2Factor.cpp
  factors/BetweenPose3Factor.cpp
  factors/BetweenQuaternionFactor.cpp
  factors/PriorFactor.cpp
  factors/RangeFactor.cpp
  factors/RangeToPointFactor.cpp
  factors/PseudorangeFactor.cpp
  factors/OdometryFactor2D.cpp
  factors/OdometryFactor2DDifferential.cpp
  factors/OdometryFactor3D.cpp
  factors/IMUFactor.cpp
  factors/IMUPreintegrationFactor.cpp
  factors/PressureDifferenceFactor.cpp
  factors/TrackingFactor.cpp
  factors/TrackingDetectionFactor.cpp
  factors/MarginalPrior.cpp
  factors/PointRegistrationFactor.cpp
  factors/BearingRangeFactor.cpp
  PARENT_SCOPE
  )

#set include directory
string(CONCAT INCLUDEDIR ${PROJECT_SOURCE_DIR}/../ "include/factors/")
