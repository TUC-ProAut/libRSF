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
 * @file AppPool_Utility.h
 * @author Tim Pfeifer
 * @date 09 Sep 2019
 * @brief Contains a set of functions related with solving the factor graph.
 * @copyright GNU Public License.
 *
 */

#ifndef APPPOOL_UTILITY_H
#define APPPOOL_UTILITY_H

#include "libRSF.h"

#include "AppPool_Adaptive.h"
#include "AppPool_Defines.h"


bool IncrementTime(const libRSF::FactorGraphConfig &Config,
                   const libRSF::SensorDataSet &Measurements,
                   double &TimeOld,
                   double &TimeNew,
                   double TimeLast);

bool GetFirstTimestamp (const libRSF::SensorDataSet &Measurements,
                        const libRSF::FactorGraphConfig &Config,
                        double &Timestamp);

bool GetLastTimestamp (const libRSF::SensorDataSet &Measurements,
                       const libRSF::FactorGraphConfig &Config,
                       double &Timestamp);

void Solve(libRSF::FactorGraph &Graph,
           const libRSF::FactorGraphConfig &Config,
           libRSF::Data & IterationSummary,
           bool ForceSolve);

void Save(libRSF::FactorGraph &Graph,
          const libRSF::FactorGraphConfig &Config,
          libRSF::Data & IterationSummary,
          libRSF::StateDataSet &Result,
          bool SaveFinal);

#endif // APPPOOL_UTILITY_H
