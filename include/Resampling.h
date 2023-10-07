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

/**
 * @file Resampling.h
 * @author Tim Pfeifer
 * @date 29.07.2019
 * @brief Helper Function to resample measurements;
 * @copyright GNU Public License.
 *
 */

#ifndef RESAMPLING_H
#define RESAMPLING_H

#include "SensorDataSet.h"
#include "Types.h"
#include "Messages.h"

namespace libRSF
{
  /** sample measurements down */
  std::vector<Data> SampleMeasurementsDown(const std::vector<Data> &Input, double SampleTime);

  Data AverageMeasurement(const std::vector<Data> &Input);
}

#endif // RESAMPLING_H
