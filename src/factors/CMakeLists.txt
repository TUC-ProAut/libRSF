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

target_sources(libRSF PRIVATE
  BaseFactor.cpp
  ConstantValueFactor.cpp
  ConstantDriftFactor.cpp
  ConstantQuaternionFactor.cpp
  ConstantPose2Factor.cpp
  BetweenValueFactor.cpp
  BetweenPose2Factor.cpp
  BetweenPose3Factor.cpp
  BetweenQuaternionFactor.cpp
  PriorFactor.cpp
  RangeFactor.cpp
  RangeToPointFactor.cpp
  PseudorangeFactor.cpp
  OdometryFactor2D.cpp
  OdometryFactor2DDifferential.cpp
  OdometryFactor3D.cpp
  IMUFactor.cpp
  IMUPreintegrationFactor.cpp
  PressureDifferenceFactor.cpp
  TrackingFactor.cpp
  TrackingDetectionFactor.cpp
  MarginalPrior.cpp
  PointRegistrationFactor.cpp
  BearingRangeFactor.cpp
)
