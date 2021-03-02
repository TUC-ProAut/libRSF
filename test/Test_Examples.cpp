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

/**
 * @file Test_ICRA19_GNSS.cpp
 * @author Leopold Mauersberger
 * @date 18 Feb 2021
 * @brief Comparing the ICRA19_GNSS application results against sample solution
 * @copyright GNU Public License.
 *
 */

#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position2D"
#include "gtest/gtest.h"

TEST(Examples, FG_Generic)
{
    /** create our own graph object */
  libRSF::FactorGraph SimpleGraph;

  /** set the solver options for ceres */
  ceres::Solver::Options SolverOptions;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = std::thread::hardware_concurrency();
  SolverOptions.minimizer_progress_to_stdout = true;

  /** add states to graph */
  SimpleGraph.addState(POSITION_STATE, libRSF::StateType::Point2, 0);
  SimpleGraph.addState(POSITION_STATE, libRSF::StateType::Point2, 1);
  SimpleGraph.addState(POSITION_STATE, libRSF::StateType::Point2, 2);
  SimpleGraph.addState(POSITION_STATE, libRSF::StateType::Point2, 3);

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
  libRSF::SensorData AbsoluteMeasurement(libRSF::SensorType::Point2, 1.0);
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
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 3.0).getNameValueString() << std::endl;

  /** load expected results */
  // reading state data set did not work like this
  libRSF::SensorDataSet Expected;

  /*
  std::vector<std::string> expectedStrings
  {
     "point2 0.0 42.0 42.0 2.0 0.25 0.25 5.0" 
  };
  Expected.addElement(libRSF::StateData("point2 0.0 42.0 42.0 2.0 0.25 0.25 5.0"));
  Expected.addElement(libRSF::StateData("point2 1.0 42.0 42.0 1.0 0.25 0.25 4.0"));
  Expected.addElement(libRSF::StateData("point2 1.0 42.0 42.0 1.0 0.25 0.25 4.0"));
  Expected.addElement(libRSF::StateData("point2 2.0 42.0 42.0 2.0 0.25 0.25 5.0"));
  Expected.addElement(libRSF::StateData("point2 3.0 42.0 42.0 3.0 0.25 0.25 6.0"));
  */
  /*
  for (auto str : expectedStrings)
  {
    Expected.addElement(libRSF::StateData(str));
  } 
  for (int i=0; i<expectedStrings.size(); ++i)
  {
    Expected.addElement(libRSF::SensorData(expectedStrings[i]));
  }
  */

  Expected.addElement(libRSF::SensorData("point2 0.0 42.0 42.0 2.0 0.25 0.25 5.0"));
// see ate for fix
  std::cout << Expected.getElement(libRSF::SensorType::Point2, 0.0).getNameValueString() << std::endl;
}

// main provided by linking to gtest_main


