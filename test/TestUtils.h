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
 * @file Statistics.h
 * @author Tim Pfeifer and Leopold Mauersberger
 * @date 02.03.2021
 * @brief Collection of testing tools.
 * @copyright GNU Public License.
 *
 */

#ifndef TESTUTILS_H
#define TESTUTILS_H

#include "libRSF.h"

namespace libRSF
{
  /** maximum component-wise absolute difference between two datasets */
  double MaxAbsError(DataType TypeGT,
                     const SensorDataSet &GT,
                     const std::string &TypeEstimate,
                     const StateDataSet &Estimate,
                     DataElement Element);

  /** RMSE of the euclidean distance of means */
  double ATE(DataType TypeGT,
             const SensorDataSet &GT,
             const std::string &TypeEstimate,
             const StateDataSet &Estimate);

  /** basic alignment*/
  void AlignTrajectory2D(const SensorDataSet &GT,
                         const std::string &PosID,
                         const std::string &RotID,
                         const StateDataSet &Estimate,
                         StateDataSet &AlignedEstimate);
}

#endif // TESTUTILS_H
