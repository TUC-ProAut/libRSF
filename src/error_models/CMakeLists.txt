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

#set source files for library
  set(SOURCEFILES ${SOURCEFILES}
  error_models/ErrorModel.cpp
  error_models/Gaussian.cpp
  error_models/GaussianComponent.cpp
  error_models/GaussianMixture.cpp
  error_models/SumMixture.cpp
  error_models/MaxMixture.cpp
  error_models/MaxSumMixture.cpp
  error_models/LossFunction.cpp
  error_models/DynamicCovarianceEstimation.cpp
  error_models/SwitchableConstraints.cpp
  PARENT_SCOPE
  )

#set include directory
string(CONCAT INCLUDEDIR ${PROJECT_SOURCE_DIR}/../ "include/error_models/")
