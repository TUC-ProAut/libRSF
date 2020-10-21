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

#include "FactorGraphSampling.h"

namespace libRSF
{
  void EvaluateCostSurfacePoints(ceres::Problem &Graph,
                                 double* const StatePointer,
                                 const int Dim,
                                 const VectorVectorSTL<Dynamic> Points,
                                 std::vector<double> &Costs)
  {

    /** save original value */
    VectorRef<double, Dynamic> State(StatePointer, Dim);
    const Vector OriginalState = State;

    /** configure evaluation */
    ceres::Problem::EvaluateOptions Options;
    Options.apply_loss_function = true;
    Options.num_threads = std::thread::hardware_concurrency();
    Options.parameter_blocks = {StatePointer};

    /** loop over grid */
    Costs.clear();
    for (int n = 0; n < static_cast<int>(Points.size()); ++n)
    {
      State = Points.at(n);

      /** evaluate */
      double Cost;
      Graph.Evaluate(Options, &Cost, nullptr, nullptr, nullptr);

      /** save result */
      Costs.push_back(Cost);
    }

    /** restore original cost */
    State = OriginalState;
  }
}
