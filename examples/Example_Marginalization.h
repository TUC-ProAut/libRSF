/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

#ifndef EXAMPLE_MARGINALIZATION_H_INCLUDED
#define EXAMPLE_MARGINALIZATION_H_INCLUDED

#include <ceres/ceres.h>
#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position"
#define ROTATION_STATE "Quaternion"

#define STDDEV_POS    0.5
#define STDDEV_ROT    0.2

void CreateData(libRSF::StateDataSet &GT,
                libRSF::SensorDataSet &Measurements);

void CreateGraphs(libRSF::FactorGraph &TranslationGraph,
                  libRSF::FactorGraph &RotationGraph,
                  libRSF::FactorGraph &PoseGraph,
                  libRSF::SensorDataSet &Measurements);

#endif // EXAMPLE_MARGINALIZATION_H_INCLUDED
