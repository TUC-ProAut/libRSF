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

#include "CalculateCovariance.h"

namespace libRSF
{

  bool CalculateCovariance(ceres::Problem &Graph,
                           StateDataSet &States,
                           const std::string &Type)
  {
    double Timestamp;

    /** create covariance object */
    ceres::Covariance::Options CovOptions;
    CovOptions.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;
    CovOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
    CovOptions.apply_loss_function = true;
    ceres::Covariance Covariance(CovOptions);

    /** create covariance block for each Position */
    std::vector<std::pair<const double*, const double*>> CovarianceBlocks;

    if (States.getTimeFirst(Type, Timestamp))
    {
      /** make pairs of pointers to each state variable */
      do
      {
        CovarianceBlocks.emplace_back(States.getElement(Type, Timestamp).getMeanPointer(),
                                   States.getElement(Type, Timestamp).getMeanPointer());
      }
      while (States.getTimeNext(Type, Timestamp, Timestamp));

      /** at first we try the more efficient algorithm */
      bool const Success = Covariance.Compute(CovarianceBlocks, &Graph);

      if (Success)
      {
        /** read covariance values to vector */
        States.getTimeFirst(Type, Timestamp);
        do
        {
          Covariance.GetCovarianceBlock(States.getElement(Type, Timestamp).getMeanPointer(),
                                        States.getElement(Type, Timestamp).getMeanPointer(),
                                        States.getElement(Type, Timestamp).getDataPointer(DataElement::Covariance));
        }
        while (States.getTimeNext(Type, Timestamp, Timestamp));
      }
      else
      {
        PRINT_ERROR("Covariance computation of ", Type, " went wrong.");
        return false;
      }
    }
    else
    {
      PRINT_ERROR("Covariance computation of ", Type, " went wrong. No ", Type, " in state data!");
      return false;
    }

    return true;
  }

  bool CalculateCovariance(ceres::Problem &Graph,
                           StateDataSet &States,
                           const std::string &Type,
                           const double Timestamp,
                           const int StateNumber)
  {

    /** create covariance object */
    ceres::Covariance::Options CovOptions;
    CovOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
    CovOptions.apply_loss_function = true;
    ceres::Covariance Covariance(CovOptions);

    /** create covariance block for each Position */
    std::vector<const double*> ParameterBlock;

    if (States.countElement(Type, Timestamp) == 1)
    {
      /** make a pair of pointers to state variable */
      ParameterBlock.push_back(States.getElement(Type, Timestamp, StateNumber).getMeanPointer());

      /** at first we try the more efficient algorithm */
      bool Success = Covariance.Compute(ParameterBlock, &Graph);

      /** if it fails, we use the more robust SVD */
      if (Success)
      {
        /** read covariance values to vector */
        Covariance.GetCovarianceBlock(States.getElement(Type, Timestamp, StateNumber).getMeanPointer(),
                                      States.getElement(Type, Timestamp, StateNumber).getMeanPointer(),
                                      States.getElement(Type, Timestamp, StateNumber).getDataPointer(DataElement::Covariance));
      }
      else if (Graph.NumParameterBlocks() < 100)
      {
        PRINT_WARNING("Jacobian related to state ", Type, " at time ", Timestamp, "s is rank-deficient. Try to compute using SVD.");


        /** re-create the covariance object with different options */
        CovOptions.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
        CovOptions.null_space_rank = -1;
        ceres::Covariance CovarianceSVD(CovOptions);

        /** try to compute again */
        Success = CovarianceSVD.Compute(ParameterBlock, &Graph);
        if (Success)
        {
          /** read covariance values to vector */
          CovarianceSVD.GetCovarianceBlock(States.getElement(Type, Timestamp, StateNumber).getMeanPointer(),
                                           States.getElement(Type, Timestamp, StateNumber).getMeanPointer(),
                                           States.getElement(Type, Timestamp, StateNumber).getDataPointer(DataElement::Covariance));
        }
      }
      else
      {
        PRINT_WARNING("Jacobian related to state ", Type, " at time ", Timestamp, "s is rank-deficient. Can't use SVD because problem is to big.");
      }

      if (!Success)
      {
        PRINT_ERROR("Covariance computation of ", Type, " at time ", Timestamp, "s went wrong.");
        return false;
      }
    }
    else
    {
      PRINT_ERROR("Covariance computation of ", Type, " went wrong. No ", Type, " in state data!");
      return false;
    }

    return true;
  }
}
