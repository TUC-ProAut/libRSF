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
 * @file Test_Example_FG_Generic.cpp
 * @author Leopold Mauersberger
 * @date 02 Mar 2021
 * @brief Comparing the FG_Generic example results against sample solution
 * @copyright GNU Public License.
 *
 */

#include "../examples/Example_FG_Generic.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position2D"

TEST(Example, FG_Generic)
{
    /** create our own graph object */
  libRSF::FactorGraph SimpleGraph;

  /** set the solver options for ceres */
  ceres::Solver::Options SolverOptions;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.minimizer_progress_to_stdout = true;

  /** add states to graph */
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 0);
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 1);
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 2);
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 3);

  /** set initial values values */
  libRSF::Vector2 StateVect;
  StateVect << 10, 10;
  SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).setMean(StateVect * 1);
  SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).setMean(StateVect * 2);
  SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).setMean(StateVect * 3);
  SimpleGraph.getStateData().getElement(POSITION_STATE, 3.0).setMean(StateVect * 4);

  /** output initialization */
  std::cout << std::endl << "Initial Values:" << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 3.0).getNameValueString() << std::endl;

  /** construct relative noise model */
  libRSF::Vector2 NoiseVect;
  NoiseVect << 1.0, 1.0;
  libRSF::GaussianDiagonal<2> NoiseRelative;
  NoiseRelative.setStdDevDiagonal(NoiseVect);

  /** add constant value factors to graph */
  SimpleGraph.addFactor<libRSF::FactorType::ConstVal2>(libRSF::StateID(POSITION_STATE, 0.0), libRSF::StateID(POSITION_STATE, 1.0), NoiseRelative);
  SimpleGraph.addFactor<libRSF::FactorType::ConstVal2>(libRSF::StateID(POSITION_STATE, 1.0), libRSF::StateID(POSITION_STATE, 2.0), NoiseRelative);
  SimpleGraph.addFactor<libRSF::FactorType::ConstVal2>(libRSF::StateID(POSITION_STATE, 2.0), libRSF::StateID(POSITION_STATE, 3.0), NoiseRelative);

  /** construct absolute noise model */
  libRSF::Matrix22 CovMat;
  CovMat << 1.0, 0.25,
            0.25, 4.0;
  libRSF::GaussianFull<2> NoisePrior;
  NoisePrior.setCovarianceMatrix(CovMat);

  /** add absolute measurement factor*/
  libRSF::Data AbsoluteMeasurement(libRSF::DataType::Point2, 1.0);
  AbsoluteMeasurement.setMean(StateVect * 4.2);
  SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 1.0), AbsoluteMeasurement, NoisePrior);

  std::cout << std::endl << "Measurement:" << std::endl;
  std::cout << AbsoluteMeasurement.getNameValueString() << std::endl;

  /** solve graph */
  std::cout << std::endl << "Solve Graph:" << std::endl;
  SimpleGraph.solve(SolverOptions);
  SimpleGraph.computeCovariance(POSITION_STATE);
  std::cout << std::endl << "Solution Report:";
  SimpleGraph.printReport();

  /** output result */
  std::cout << std::endl << "Optimized Values:" << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 3.0).getNameValueString() << std::endl;

  /** load expected results */
  libRSF::SensorDataSet Expected;
  Expected.addElement(libRSF::Data("point2 0.0 42.0 42.0 2.0 0.25 0.25 5.0"));
  Expected.addElement(libRSF::Data("point2 1.0 42.0 42.0 1.0 0.25 0.25 4.0"));
  Expected.addElement(libRSF::Data("point2 2.0 42.0 42.0 2.0 0.25 0.25 5.0"));
  Expected.addElement(libRSF::Data("point2 3.0 42.0 42.0 3.0 0.25 0.25 6.0"));

  /** calculate maximum component-wise absolute difference between solution and expected (mean and covariance) */
  const double maxAbsErrorMean = libRSF::MaxAbsError(libRSF::DataType::Point2,
                                                     Expected,
                                                     POSITION_STATE,
                                                     SimpleGraph.getStateData(),
                                                     libRSF::DataElement::Mean);
  const double maxAbsErrorCov = libRSF::MaxAbsError(libRSF::DataType::Point2,
                                                    Expected, POSITION_STATE,
                                                    SimpleGraph.getStateData(),
                                                    libRSF::DataElement::Covariance);

  std::cout << "MaxAbsErrorMean:" << maxAbsErrorMean << std::endl;
  std::cout << "MaxAbsErrorCov:" << maxAbsErrorCov << std::endl;

  EXPECT_LT(maxAbsErrorMean,1e-3);
  EXPECT_LT(maxAbsErrorCov,1e-3);
}

/** main provided by linking to gtest_main */
