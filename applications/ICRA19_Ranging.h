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

#ifndef ICRA19_RANGING_H_INCLUDED
#define ICRA19_RANGING_H_INCLUDED

#include "libRSF.h"

#include <cstdio>
#include <cstring>

#define POSITION_STATE "Position"
#define ORIENTATION_STATE "Orientation"


/** Adds a range measurement to the graph of a 2D pose estimation problem */
void AddRangeMeasurements2D(libRSF::FactorGraph &Graph,
                            libRSF::SensorDataSet &Measurements,
                            libRSF::FactorGraphConfig const &Config,
                            double Timestamp);

/** use EM algorithm to tune the gaussian mixture model */
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config,
                    int NumberOfComponents);

/** parse string from command line to select error model for ranging*/
bool ParseErrorModel(const std::string &ErrorModel, libRSF::FactorGraphConfig &Config);

#endif // ICRA19_RANGING_H_INCLUDED
