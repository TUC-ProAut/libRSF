/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
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

/**
 * @file Statistics.h
 * @author Tim Pfeifer and Leopold Mauersberger
 * @date 02.03.2021
 * @brief Collection of testing tools.
 * @copyright GNU Public License.
 *
 */

#ifndef TESTUTILS_H
#define TESTUTILS_H

#include "VectorMath.h"
#include "StateDataSet.h"
#include "SensorDataSet.h"

namespace libRSF
{
  double RMSE(Vector V);

  // maximum componentwise absolute difference between two datasets (mean and covariance)
  double MaxAbsError(SensorType TypeGT,
             SensorElement ElementGT,
             SensorDataSet GT,
             std::string TypeEstimate,
             StateElement ElementEstimate,
             StateDataSet Estimate);

  double ATE(SensorType TypeGT,
             SensorDataSet GT,
             std::string TypeEstimate,
             StateDataSet Estimate);
}

#endif // TESTUTILS_H