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
 * @file FactorGraphSampling.h
 * @author Tim Pfeifer
 * @date 02.11.2017
 * @brief Different functions to sample the residual surface of the optimization problem.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORGRAPHSAMPLING_H
#define FACTORGRAPHSAMPLING_H

#include "StateDataSet.h"

#include <ceres/ceres.h>
#include <thread>

namespace libRSF
{
  void EvaluateCostSurfacePoints(ceres::Problem &Graph,
                                 double * StatePointer,
                                 int Dim,
                                 const VectorVectorSTL<Dynamic>& Points,
                                 std::vector<double> &Costs);


  template<int Dim>
  void EvaluateCostSurface(ceres::Problem &Graph,
                           double * StatePointer,
                           const int Points,
                           const double Range,
                           StateDataSet &Result,
                           const bool OptimizeOtherStates = false)
  {
    using SizedVector = VectorStatic<Dim>;

    /** configure evaluation */
    ceres::Problem::EvaluateOptions Options;
    Options.apply_loss_function = true;
    Options.num_threads = static_cast<int>(std::thread::hardware_concurrency());
    Options.parameter_blocks = {StatePointer};

    /** save original value */
    VectorRef<double, Dim> State(StatePointer);
    const SizedVector OriginalState = State;
    if(OptimizeOtherStates)
    {
      Graph.SetParameterBlockConstant(StatePointer);
    }

    /** create 1D point set */
    Vector Lin = Vector::LinSpaced(Points, -Range/2, Range/2);

    /** create nD point set */
    std::vector<Vector> Grid;
    for (int nDim = 0; nDim < Dim; ++nDim)
    {
      Grid.push_back((Lin.array() + State(nDim)).matrix());
    }

    /** create multi dimensional index */
    std::vector<int> Index;
    for (int nDim = 0; nDim < Dim; ++nDim)
    {
      Index.push_back(0);
    }

    /** loop over grid */
    for (uint64_t n = 0; n < static_cast<uint64_t>(std::pow(Points,Dim)); ++n)
    {
      /** write point */
      for (int nDim = 0; nDim < Dim; ++nDim)
      {
        State.segment(nDim,1) = Grid.at(nDim).segment(Index.at(nDim),1);
      }

      /** optimize if required (for complex error models with free variables like SC or DCE) */
      if(OptimizeOtherStates)
      {
        ceres::Solver::Options Options;
        ceres::Solver::Summary Summary;
        Options.num_threads = static_cast<int>(std::thread::hardware_concurrency());
        ceres::Solve(Options, &Graph, &Summary);
      }

      /** evaluate */
      double Cost;
      std::vector<double> Gradient;
      ceres::CRSMatrix JacobianCRS;
      Graph.Evaluate(Options, &Cost, nullptr, &Gradient, &JacobianCRS);

      /** calculate the Hessian */
      Matrix Jacobian;
      CRSToMatrix(JacobianCRS, Jacobian);
      MatrixStatic<Dim,Dim> Hessian = Jacobian.transpose()*Jacobian;

      /** create state data to store the result */
      Data CostState;
      if (Dim == 1)
      {
        CostState = Data(DataType::CostGradient1, 0.0);
      }
      else if (Dim == 2)
      {
        CostState = Data(DataType::CostGradient2, 0.0);
      }
      else if (Dim == 3)
      {
        CostState = Data(DataType::CostGradient3, 0.0);
      }
      else
      {
        PRINT_ERROR("There is no data type for ", Dim, " dimensional data!");
      }

      /** point where the cost is evaluated */
      CostState.setMean(State);

      /** actual cost value */
      CostState.setValueScalar(DataElement::Cost, Cost);

      /** gradient of the cost surface */
      SizedVector GradientVector(Gradient.data());
      CostState.setValue(DataElement::Gradient, GradientVector);

      /** hessian at the evaluated point */
      VectorStatic<Dim*Dim> HessianVector(Hessian.data());
      CostState.setValue(DataElement::Hessian, HessianVector);

      /** push to data set */
      Result.addElement(CostState);

      /**increment indexes */
      Index.at(0)++;
      for (int i = 0; i < Dim; ++i)
      {
        /** if one index is maxed */
        if (Index.at(i) >= Points && i < (Dim-1))
        {
          /** reset current index */
          Index.at(i) = 0;
          /** increment next index level */
          Index.at(i+1)++;
        }
      }
    }

    /** restore original value */
    State = OriginalState;
    if(OptimizeOtherStates)
    {
      Graph.SetParameterBlockVariable(StatePointer);
    }
  }
}

#endif // FACTORGRAPHSAMPLING_H
