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
 * @file AppPol_Init.h
 * @author Tim Pfeifer
 * @date 19 August 2019
 * @brief Contains a set of functions to calculate good initial values for the factor graph.
 * @copyright GNU Public License.
 *
 */

#ifndef APPPOOL_INIT_H
#define APPPOOL_INIT_H

#include "AppPool_Sensors.h"
#include "AppPool_Defines.h"

#include "libRSF.h"

#include <algorithm>

/** use the current set of GNSS measurements to estimate position based on random sample consensus  */
void PseudoRangeRANSAC(libRSF::FactorGraph &Graph,
                       const libRSF::FactorGraphConfig &Config,
                       const libRSF::SensorDataSet &Measurements,
                       double TimeStart,
                       double TimeEnd,
                       int Iterations,
                       double Threshold);

/** sample state around the current mean to get a good initial guess */
void PseudoRangeSampling(libRSF::FactorGraph &Graph,
                         double Time,
                         int Iterations,
                         double SamplingStdDev);

/** use the first set of GNSS pseudo ranges to estimate position */
 void InitWithGNSS(libRSF::FactorGraph &Graph,
                  libRSF::SensorDataSet &Measurements,
                  const libRSF::FactorGraphConfig &Config,
                  libRSF::TangentPlaneConverter &LocalFrame,
                  double TimeInitial,
                  double WindowLength);

/** use the first set of UWB ranges to estimate position */
 void InitWithUWB(libRSF::FactorGraph &Graph,
                  libRSF::SensorDataSet &Measurements,
                  const libRSF::FactorGraphConfig &Config,
                  double TimeInitial,
                  double WindowLength,
                  int MinRangeNumber);

/** use the first set of IMU measurements to align gravity and to estimate bias */
void InitIMU(libRSF::FactorGraph &Graph,
             libRSF::SensorDataSet &Measurements,
             double TimeInitial,
             double WindowLength);

/** add a generic "up-right" prior */
void InitOdom(libRSF::FactorGraph &Graph,
              libRSF::FactorType OdomType,
              double TimeInitial);

#endif // APPPOOL_INIT_H
