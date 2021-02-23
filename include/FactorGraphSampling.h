/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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

#ifndef ROBUSTOPTIMIZATION_H
#define ROBUSTOPTIMIZATION_H

#include "StateDataSet.h"

#include <ceres/ceres.h>
#include <thread>

namespace libRSF
{
  void EvaluateCostSurfacePoints(ceres::Problem &Graph,
                                 double * const StatePointer,
                                 const int Dim,
                                 const VectorVectorSTL<Dynamic> Points,
                                 std::vector<double> &Costs);


  template<int Dim>
  void EvaluateCostSurface(ceres::Problem &Graph,
                           double * const StatePointer,
                           const int Points,
                           const double Range,
                           StateDataSet &Result)
  {
    typedef VectorStatic<Dim> SizedVector;

    /** configure evaluation */
    ceres::Problem::EvaluateOptions Options;
    Options.apply_loss_function = true;
    Options.num_threads = std::thread::hardware_concurrency();
    Options.parameter_blocks = {StatePointer};

    /** save original value */
    VectorRef<double, Dim> State(StatePointer);
    const SizedVector OriginalState = State;

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
    for (uint64_t n = 0; n < std::pow(Points,Dim); ++n)
    {
      /** write point */
      for (int nDim = 0; nDim < Dim; ++nDim)
      {
        State.segment(nDim,1) = Grid.at(nDim).segment(Index.at(nDim),1);
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
      StateData CostState(StateType::CostGradient, 0.0);
      CostState.resizeElement(StateElement::Mean, Dim);
      CostState.resizeElement(StateElement::Cost, 1);
      CostState.resizeElement(StateElement::Gradient, Dim);
      CostState.resizeElement(StateElement::Hessian, Dim*Dim);

      /** point where the cost is evaluated */
      CostState.setMean(State);

      /** actual cost value */
      CostState.setValue(StateElement::Cost, (Vector1() << Cost).finished());

      /** gradient of the cost surface */
      SizedVector GradientVector(Gradient.data());
      CostState.setValue(StateElement::Gradient, GradientVector);

      /** hessian at the evaluated point */
      VectorStatic<Dim*Dim> HessianVector(Hessian.data());
      CostState.setValue(StateElement::Hessian, HessianVector);

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
  }
}

#endif // ROBUSTOPTIMIZATION_H
