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

#include "TestUtils.h"

namespace libRSF
{

  /** \brief RMSE between two datasets (mean)
   *
   * \param TypeGT the type of the GT data
   * \param GT a data set that holds the GT
   * \param TypeEstimate type of the estimate
   * \param Estimate data set that holds the estimation
   * \return root mean square error between GT and estimate
   */
  double ATE(const DataType TypeGT,
             const SensorDataSet &GT,
             const std::string &TypeEstimate,
             const StateDataSet &Estimate)
  {
    /** check length */
    const int LengthGT = GT.countElements(TypeGT);
    const int LengthEstimate = Estimate.countElements(TypeEstimate);

    /** check for zero length, separately */
    if (LengthGT == 0)
    {
      PRINT_ERROR("Length of GT is zero!");
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (LengthEstimate == 0)
    {
      PRINT_ERROR("Length of estimate is zero!");
      return std::numeric_limits<double>::quiet_NaN();
    }

    /** assign timestamps if not equal */
    if (LengthGT != LengthEstimate)
    {
      PRINT_WARNING("Length of GT and estimate are not equal. Using closest GT points.");
      PRINT_LOGGING("Length of GT: " + std::to_string(LengthGT));
      PRINT_LOGGING("Length of estimate: " + std::to_string(LengthEstimate));
    }

    /** vector to store errors */
    Vector Error(LengthEstimate);

    /** get first timestamp */
    double Time = 0.0;
    Estimate.getTimeFirst(TypeEstimate, Time);

    /** fill error vector */
    int n = 0;
    do
    {
      /** get estimate at this timestamp */
      Data DataEstimate;
      Estimate.getElement(TypeEstimate, Time, 0, DataEstimate);

      /** find closest GT */
      double TimeGT = 0.0;
      GT.getTimeCloseTo(TypeGT, Time, TimeGT);
      Data DataGT;
      GT.getElement(TypeGT, TimeGT, 0, DataGT);

      /** euclidean distance*/
      Error(n) = (DataEstimate.getMean() - DataGT.getMean()).norm();
      n++;
    }
    while(Estimate.getTimeNext(TypeEstimate, Time, Time));

    /** apply RMSE */
    return RMSE(Error);
  }


  /** \brief maximum component-wise absolute difference between two datasets (mean and covariance)
   *
   * \param TypeGT the type of the GT data
   * \param GT a data set that holds the GT
   * \param TypeEstimate type of the estimate
   * \param Estimate data set that holds the estimation
   * \return maximum absolute error between GT and estimate
   */
  double MaxAbsError(const DataType TypeGT,
                     const SensorDataSet &GT,
                     const std::string &TypeEstimate,
                     const StateDataSet &Estimate,
                     const DataElement Element)
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
    double Time = 0.0;
    GT.getTimeFirst(TypeGT, Time);

    /** calculate overall maximum */
    do
    {
      int NumberOfStates = GT.countElement(TypeGT,Time);
      for (int nState = 0; nState < NumberOfStates; ++nState)
      {
        /** get data at this timestamp */
        Data DataGT, DataEstimate;
        Estimate.getElement(TypeEstimate, Time, nState, DataEstimate);
        GT.getElement(TypeGT, Time, nState, DataGT);

        /** get maximum difference */
        const double maxAbsErrorTmp = (DataEstimate.getValue(Element) - DataGT.getValue(Element)).cwiseAbs().maxCoeff();

        /** store maximum of loop */
        if(maxAbsErrorTmp > maxAbsError)
        {
          maxAbsError = maxAbsErrorTmp;
        }
      }
    }
    while(GT.getTimeNext(TypeGT, Time, Time));

    /** return overall maximum */
    return maxAbsError;
  }

  void AlignTrajectory2D(const SensorDataSet &GT,
                         const std::string &PosID,
                         const std::string &RotID,
                         const StateDataSet &Estimate,
                         StateDataSet &AlignedEstimate)
  {
      /** check length */
      const int LengthGT = GT.countElements(DataType::Point2);
      const int LengthEstimate = Estimate.countElements(PosID);

      /** align rotation first */
      Vector RotDiff(LengthEstimate);
      int n = 0;
      for(const Data &Rot : Estimate.getElementsOfID(RotID))
      {
        /** find GT time */
        double TimeGT = 0.0;
        GT.getTimeCloseTo(DataType::Angle, Rot.getTimestamp(), TimeGT);
        Data RotGT;
        GT.getElement(DataType::Angle, TimeGT, 0, RotGT);

        RotDiff(n) = NormalizeAngle(RotGT.getMean()(0)  - Rot.getMean()(0));
        n++;
      }

      /** use median of relative rotation because mean is more complicated */
      const Matrix22 RotationMatrix = RotationMatrix2D(Median(RotDiff));

      /** apply rotation*/
      for(const Data &Pos : Estimate.getElementsOfID(PosID))
      {
        /** rotate estimate */
        Data PosNew;
        PosNew = Pos;
        PosNew.setMean(RotationMatrix * Pos.getMean());
        PosNew.setCovarianceMatrix(RotationMatrix * Pos.getCovarianceMatrix() * RotationMatrix.transpose());

        /** copy in new dataset */
        AlignedEstimate.addElement(PosID, PosNew);
      }

      /** find transformation */
      Matrix PosDiff(LengthEstimate,2);
      n = 0;
      for(const Data &Pos : AlignedEstimate.getElementsOfID(PosID))
      {
        /** find GT time */
        double TimeGT = 0.0;
        GT.getTimeCloseTo(DataType::Point2, Pos.getTimestamp(), TimeGT);
        Data PosGT;
        GT.getElement(DataType::Point2, TimeGT, 0, PosGT);

        /** evaluate position difference */
        PosDiff(n, 0) = PosGT.getMean()(0) - Pos.getMean()(0);
        PosDiff(n, 1) = PosGT.getMean()(1) - Pos.getMean()(1);
        n++;
      }
      const Vector2 Translation = PosDiff.colwise().mean();

      /** apply translation*/
      for(const Data& Pos : AlignedEstimate.getElementsOfID(PosID))
      {
        /** translate estimate */
        AlignedEstimate.getElement(PosID, Pos.getTimestamp(), 0).setMean(Pos.getMean() + Translation);
      }
  }
}
