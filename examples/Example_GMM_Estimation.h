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
 * @file Example_GMM_Estimation.h
 * @author Tim Pfeifer
 * @date 29.11.2021
 * @brief Helper functions for the GMM estimation examples and tests.
 * @copyright GNU Public License.
 *
 */

#ifndef EXAMPLE_GMM_1D_H
#define EXAMPLE_GMM_1D_H

#include <random>
#include <chrono>

#include "libRSF.h"

template <int Dim>
libRSF::MatrixStatic<Dim,libRSF::Dynamic> GenerateSamplesGMM(const int Number,
                                                             const libRSF::VectorStatic<Dim> &Mean1,
                                                             const libRSF::MatrixStatic<Dim,Dim> &Cov1,
                                                             const libRSF::Vector1 &Weight1,
                                                             const libRSF::VectorStatic<Dim>  &Mean2,
                                                             const libRSF::MatrixStatic<Dim,Dim> &Cov2,
                                                             const libRSF::Vector1 &Weight2)
{
  /** normalize Weights */
  const double WeightNorm = Weight1(0)/(Weight1(0)+Weight2(0));

  /** prepare sample transformation */
  Eigen::SelfAdjointEigenSolver<libRSF::MatrixStatic<Dim,Dim>> Solver1(Cov1);
  Eigen::SelfAdjointEigenSolver<libRSF::MatrixStatic<Dim,Dim>> Solver2(Cov2);

  const libRSF::MatrixStatic<Dim,Dim> Transform1 = Solver1.eigenvectors() * Solver1.eigenvalues().cwiseSqrt().asDiagonal();
  const libRSF::MatrixStatic<Dim,Dim> Transform2 = Solver2.eigenvectors() * Solver2.eigenvalues().cwiseSqrt().asDiagonal();

  /** set up Gaussian generator */
  unsigned int Seed = 12345;
  std::default_random_engine Generator (Seed);
  std::normal_distribution<double> Gaussian(0, 1);

  /** crate storage */
  libRSF::MatrixStatic<Dim,libRSF::Dynamic> Samples(Dim,Number);

  /** draw samples */
  for (int n = 0; n < Number; ++n)
  {
    libRSF::VectorStatic<Dim> SingleSample;
    for (int d = 0; d < Dim; ++d)
    {
      SingleSample(d) = Gaussian(Generator);
    }

    /** switch between component 1 and 2 */
    if(n < Number*WeightNorm)
    {
      SingleSample = Transform1 * SingleSample + Mean1;
    }
    else
    {
      SingleSample = Transform2 * SingleSample + Mean2;
    }

    Samples.col(n) = SingleSample;
  }

  return Samples;
}

#endif // EXAMPLE_GMM_1D_H
