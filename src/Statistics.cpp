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

#include "Statistics.h"

namespace libRSF
{
  double Median(const std::vector<double> &V)
  {
    std::vector<double> VCopy = V;
    int n = static_cast<int>(VCopy.size()) / 2;
    std::nth_element(VCopy.begin(), VCopy.begin() + n, VCopy.end());
    return VCopy[n];
  }

  double Median(const Vector &V)
  {
    std::vector<double> Vec(V.data(), V.data() + V.rows() * V.cols());
    return Median(Vec);
  }

  double MAD(const Vector &V)
  {
    return Median((V.array() - Median(V)).abs().matrix());
  }

  double EstimateMADCovariance(const Vector &V)
  {
    return MAD(V) * 1.4826;
  }

  double RMSE(const Vector &V)
  {
    return sqrt(V.squaredNorm() / static_cast<double>(V.size()));
  }
}
