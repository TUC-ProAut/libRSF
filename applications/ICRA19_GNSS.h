/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
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

#ifndef ICRA19_GNSS_H_INCLUDED
#define ICRA19_GNSS_H_INCLUDED

#include <stdio.h>
#include <string.h>

#include <ceres/ceres.h>
#include "../include/FactorGraph.h"
#include "../include/FactorGraphConfig.h"
#include "../include/FileAccess.h"
#include "../include/StateDataSet.h"
#include "../include/SensorDataSet.h"


#define POSITION_STATE "Position"
#define ORIENTATION_STATE "Orientation"
#define CLOCK_ERROR_STATE "ClockError"
#define CLOCK_DRIFT_STATE "ClockDrift"

/** Generates a delta time measurement object from two timestamps */
libRSF::SensorData GenerateDeltaTime(const double TimestampOld,
                                      const double TimestampNew);

/** Adds a pseudorange measurement to the graph */
void AddPseudorangeMeasurements(libRSF::FactorGraph& Graph,
                                libRSF::SensorDataSet & Measurements,
                                libRSF::FactorGraphConfig const &Config,
                                double Timestamp);

/** use EM algorithm to tune the gaussian mixture model */
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config,
                    int NumberOfComponents,
                    double Timestamp);

#endif // ICRA19_GNSS_H_INCLUDED
