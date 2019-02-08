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

/**
 * @file PredictConstantDrift.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Predictor for a constant drift model.
 * @copyright GNU Public License.
 *
 */

#ifndef PREDICTCONSTANTDRIFT_H
#define PREDICTCONSTANTDRIFT_H

#include <ceres/ceres.h>

namespace libRSF
{
  template <typename T, int Dimension>
  void PredictConstantDrift(const T* const Vector1, T* const Vector2, const T* const Drift1, T* const Drift2, const double &DeltaTime)
  {
    for(int nDim = 0; nDim < Dimension; nDim++)
    {
      Vector2[nDim] = Vector1[nDim] + Drift1[nDim] * DeltaTime;
      Drift2[nDim] = Drift1[nDim];
    }
  }
}

#endif // PREDICTCONSTANTDRIFT_H
