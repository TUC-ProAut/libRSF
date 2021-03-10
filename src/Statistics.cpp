/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
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
  double Median(std::vector<double> &V)
  {
    int n = V.size() / 2;
    std::nth_element(V.begin(), V.begin() + n, V.end());
    return V[n];
  }

  double Median(Vector V)
  {
    std::vector<double> Vec(V.data(), V.data() + V.rows() * V.cols());
    return Median(Vec);
  }

  double MAD(Vector V)
  {
    return Median((V.array() - Median(V)).abs().matrix());
  }

  double RMSE(Vector V)
  {
    return sqrt(V.squaredNorm() / V.size());
  }

  double ATE(DataType TypeGT,
             SensorDataSet GT,
             std::string TypeEstiamte,
             StateDataSet Estimate)
  {
    /** check length */
    const int LengthGT = GT.countElements(TypeGT);
    const int LengthEstimate = Estimate.countElements(TypeEstiamte);

    /** return error if not equal */
    if (LengthGT != LengthEstimate)
    {
      PRINT_ERROR("Length of GT and estimate is not identical!");
      return std::numeric_limits<double>::quiet_NaN();
    }

    /** vector to store errors */
    Vector Error(LengthEstimate);

    /** get first timestamp */
    double Time;
    GT.getTimeFirst(TypeGT, Time);

    /** fill error vector */
    int n = 0;
    do
    {
      Error(n) = (Estimate.getElement(TypeEstiamte, Time).getMean() - GT.getElement(TypeGT, Time).getMean()).norm();
      n++;
    }
    while(GT.getTimeNext(TypeGT, Time, Time));

    /** apply RMSE */
    return RMSE(Error);
  }
}
