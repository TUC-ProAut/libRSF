/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option); any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de);
 ***************************************************************************/

/**
 * @file libRSF.h
 * @author Tim Pfeifer
 * @date 09 August 2019
 * @brief Central Header to include all relevant functions of libRSF.
 * @copyright GNU Public License.
 *
 */

#ifndef LIBRSF_H_INCLUDED
#define LIBRSF_H_INCLUDED

/** most important functions */
#include "FactorGraph.h"
#include "FactorGraphConfig.h"
#include "FileAccess.h"
#include "Misc.h"
#include "StateDataSet.h"
#include "SensorDataSet.h"
#include "GNSS.h"
#include "Resampling.h"
#include "TimeMeasurement.h"
#include "geometric_models/OdometryIntegrator.h"
#include "geometric_models/IMUPreintegrator.h"
#include "Statistics.h"

#endif // LIBRSF_H_INCLUDED
