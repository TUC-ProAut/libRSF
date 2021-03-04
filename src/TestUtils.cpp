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

#include "TestUtils.h"

namespace libRSF
{
  double RMSE(Vector V)
  {
    return sqrt(V.squaredNorm() / V.size());
  }

  double ATE(SensorType TypeGT,
             SensorDataSet GT,
             std::string TypeEstimate,
             StateDataSet Estimate)
  {
    /** check length */
    const int LengthGT = GT.countElements(TypeGT);
    const int LengthEstimate = Estimate.countElements(TypeEstimate);

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
      Error(n) = (Estimate.getElement(TypeEstimate, Time).getMean() - GT.getElement(TypeGT, Time).getMean()).norm();
      n++;
    }
    while(GT.getTimeNext(TypeGT, Time, Time));

    /** apply RMSE */
    return RMSE(Error);
  }

  // maximum componentwise absolute difference between two datasets (mean and covariance)
  double MaxAbsError(SensorType TypeGT,
             SensorElement ElementGT,
             SensorDataSet GT,
             std::string TypeEstimate,
             StateElement ElementEstimate,
             StateDataSet Estimate)
  {
    /** check length */
    const int LengthGT = GT.countElements(TypeGT);
    const int LengthEstimate = Estimate.countElements(TypeEstimate);

    /** return error if not equal */
    if (LengthGT != LengthEstimate)
    {
      PRINT_ERROR("Length of GT and estimate is not identical!");
      return std::numeric_limits<double>::quiet_NaN();
    }

    /** initialize maximum error */
    double maxAbsError = 0;

    /** get first timestamp */
    double Time;
    GT.getTimeFirst(TypeGT, Time);

    /** calculate overall maximum */
    do
    {
      double maxAbsErrorMean = (Estimate.getElement(TypeEstimate, Time).getValue(ElementEstimate) - GT.getElement(TypeGT, Time).getValue(ElementGT)).cwiseAbs().maxCoeff();
      if(maxAbsErrorMean > maxAbsError) maxAbsError = maxAbsErrorMean;
    }
    while(GT.getTimeNext(TypeGT, Time, Time));

    /** return overall maximum */
    return maxAbsError;
  }
}