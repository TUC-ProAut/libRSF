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
 * @file CalculateCovariance.h
 * @author Tim Pfeifer
 * @date 2017
 * @brief Functions that allow to use ceres' covariance estimation capabilities.
 * @copyright GNU Public License.
 *
 */

#ifndef CALCULATECOVARIANCE_H
#define CALCULATECOVARIANCE_H

#include "FactorGraphSampling.h"
#include "VectorTypes.h"
#include "StateDataSet.h"
#include "Messages.h"
#include "Tensor.h"

#include <ceres/ceres.h>

#include <vector>
#include <thread>

namespace libRSF
{
  /** @brief Calculates the Covariance of one datatype over a complete dataset.
   *
   * @param ceres::Problem& Graph The graph that contains the factors.
   * @param libRSF::StateDataSet &States Struct that contains the data of the Graph. The Covariance is saved here!
   * @param std::string Type Specific identifier for the desired datatype. (e.g. Position, Velocity...)
   * @return true if everything works fine.
   *
   */
  bool CalculateCovariance(ceres::Problem &Graph,
                           StateDataSet &States,
                           const std::string &Type);


  /** @brief Calculates the Covariance of one datatype for a specific timestamp.
   *
   * @param ceres::Problem& Graph The graph that contains the factors.
   * @param libRSF::StateDataSet &States Struct that contains the data of the Graph. The Covariance is saved here!
   * @param std::string Type Specific identifier for the desired datatype. (e.g. Position, Velocity...)
   * @param double Timestamp Desired Timestamp.
   * @return true if everything works fine.
   *
   */
  bool CalculateCovariance(ceres::Problem &Graph,
                           StateDataSet &States,
                           const std::string &Type,
                           double Timestamp,
                           int StateNumber = 0);


  /** \brief Compute the covariance based on a numerical estimation of the Hessian
   *
   * \param Grid<Dim, 3> CostGrid A class that holds the evaluated cost surface with multiple (Dim) dimensions.
   * \param VectorStatic<Dim> Delta Spacing of the grid points.
   * \return The estimated covariance matrix.
   *
   */
  template<int Dim>
  MatrixStatic<Dim, Dim> NumericalCovariance(const Tensor<Dim, 3> &CostGrid, const VectorStatic<Dim> &Delta)
  {
    MatrixStatic<Dim, Dim> Hessian = MatrixStatic<Dim, Dim>::Zero();

    /** fill Hessian */
    for (int nDim1 = 0; nDim1 < Dim; nDim1++)
    {
      VectorStatic<Dim> Index = VectorStatic<Dim>::Ones();

      /** line for variance */
      Index(nDim1) = 0;
      Hessian(nDim1, nDim1) = CostGrid.get(Index);
      Index(nDim1) = 2;
      Hessian(nDim1, nDim1) += CostGrid.get(Index);
      Index(nDim1) = 1;
      Hessian(nDim1, nDim1) = (Hessian(nDim1, nDim1) - 2 * CostGrid.get(Index)) / (Delta(nDim1) * Delta(nDim1));

      /** square for covariance */
      for (int nDim2 = nDim1 + 1; nDim2 < Dim; nDim2++)
      {
        Index = VectorStatic<Dim>::Ones();

        Index(nDim1) = 0;
        Index(nDim2) = 0;
        Hessian(nDim1, nDim2) = CostGrid.get(Index);

        Index(nDim1) = 0;
        Index(nDim2) = 2;
        Hessian(nDim1, nDim2) -= CostGrid.get(Index);

        Index(nDim1) = 2;
        Index(nDim2) = 0;
        Hessian(nDim1, nDim2) -= CostGrid.get(Index);

        Index(nDim1) = 2;
        Index(nDim2) = 2;
        Hessian(nDim1, nDim2) += CostGrid.get(Index);

        Hessian(nDim1, nDim2) /= (4 * Delta(nDim1) * Delta(nDim2));

        /** use symmetry */
        Hessian(nDim2, nDim1) = Hessian(nDim1, nDim2);
      }
    }

    /** Covariance = Hessian^-1 */
    return Hessian.completeOrthogonalDecomposition().pseudoInverse();
  }

  /** @brief Calculates the Covariance of one datatype for a specific timestamp using the sigma point algorithm.
   *
   * @param ceres::Problem& Graph The graph that contains the factors.
   * @param libRSF::StateDataSet &States Struct that contains the data of the Graph. The Covariance is saved here!
   * @param std::string StateName Specific identifier for the desired datatype. (e.g. Position, Velocity...)
   * @param double Timestamp Desired Timestamp.
   * @return true if everything works fine.
   *
   */
  template<int Dim>
  bool EstimateCovarianceSigmaPoint(ceres::Problem &Graph,
                                    StateDataSet &States,
                                    const std::string &StateName,
                                    const double StateTimestamp,
                                    const int StateNumber)
  {
    /** estimate covariance with Hessian */
    bool Success = CalculateCovariance(Graph, States, StateName, StateTimestamp, StateNumber);

    if (Success)
    {
      /** get initial information about the state */
      double* const StatePointer = States.getElement(StateName, StateTimestamp, StateNumber).getMeanPointer();
      const VectorStatic<Dim> Mean = States.getElement(StateName, StateTimestamp, StateNumber).getMean();
      const MatrixStatic<Dim, Dim> StdDev = SquareRoot<Dim, double>(States.getElement(StateName, StateTimestamp, StateNumber).getCovarianceMatrix());
      const VectorStatic<Dim> Delta = StdDev.diagonal();

      /** create the required evaluation points */
      VectorVectorSTL<Dim> Indices;
      VectorVectorSTL<Dynamic> Points;

      /** add center point */
      Indices.emplace_back(VectorStatic<Dim>::Ones());

      /** add other points */
      for (int nDim1 = 0; nDim1 < Dim; nDim1++)
      {
        VectorStatic<Dim> Index = VectorStatic<Dim>::Ones();

        /** line for variance */
        Index(nDim1) = 0;
        Indices.emplace_back(Index);
        Index(nDim1) = 2;
        Indices.emplace_back(Index);

        /** square for covariance */
        for (int nDim2 = nDim1 + 1; nDim2 < Dim; nDim2++)
        {
          Index = VectorStatic<Dim>::Ones();

          Index(nDim1) = 0;
          Index(nDim2) = 0;
          Indices.emplace_back(Index);

          Index(nDim2) = 2;
          Indices.emplace_back(Index);

          Index(nDim1) = 2;
          Index(nDim2) = 0;
          Indices.emplace_back(Index);

          Index(nDim2) = 2;
          Indices.emplace_back(Index);
        }
      }

      /** convert indexes to points */
      for (const VectorStatic<Dim> &Index : Indices)
      {
        VectorStatic<Dim> Offset = (Index - VectorStatic<Dim>::Ones()).array() * Delta.array();
        Points.emplace_back(Mean + Offset);
      }

      /** evaluate sigma points */
      std::vector<double> Cost;
      EvaluateCostSurfacePoints(Graph, StatePointer, Dim, Points, Cost);

      /** parse them into a tensor */
      Tensor<Dim, 3> CostTensor;
      for (int nPoint = 0; nPoint < static_cast<int>(Points.size()); nPoint++)
      {
        CostTensor.set(Cost.at(nPoint), Indices.at(nPoint));
      }

      /** estimate covariance again */
      const MatrixStatic<Dim, Dim> Cov = NumericalCovariance<Dim>(CostTensor, Delta);
      const VectorRefConst<double, Dim* Dim> CovVect(Cov.data());

      /** overwrite state covariance */
      States.getElement(StateName, StateTimestamp, StateNumber).setCovariance(CovVect);
    }

    return Success;
  }
}
#endif // CALCULATECOVARIANCE_H
