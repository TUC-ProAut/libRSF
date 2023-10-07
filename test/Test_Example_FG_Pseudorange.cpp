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
 * @file Test_Example_FG_Pseudorange.cpp
 * @author Leopold Mauersberger
 * @date 02 Mar 2021
 * @brief Comparing the FG_Pseudorange example results against sample solution
 * @copyright GNU Public License.
 *
 */

#include "../examples/Example_FG_Pseudorange.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

TEST(Example, FG_Pseudorange)
{
  double Time, TimeFirst = 0.0, TimeOld = 0.0;

  libRSF::Vector1 StdDev, StdDevCCE;
  StdDev << STDDEV_RANGE;
  StdDevCCE << STDDEV_CCE;

  ceres::Solver::Options SolverOptions;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.minimizer_progress_to_stdout = true;

  libRSF::FactorGraph SimpleGraph;
  libRSF::SensorDataSet RangeMeasurements;
  libRSF::StateList RangeList, ConstValList;
  libRSF::GaussianDiagonal<1> NoiseModelRange, NoiseModelCCE;
  NoiseModelRange.setStdDevDiagonal(StdDev);
  NoiseModelCCE.setStdDevDiagonal(StdDevCCE);

  /** construct a set of range Measurements */
  CreateData(RangeMeasurements);

  /** loop over timestamps */
  RangeMeasurements.getTimeFirst(PSEUDORANGE_MEASUREMENT, TimeFirst);
  Time = TimeFirst;
  do
  {
    /** add position variables to graph */
    SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, Time);

    /** add offset variables to graph */
    SimpleGraph.addState(OFFSET_STATE, libRSF::DataType::ClockError, Time);

    RangeList.add(POSITION_STATE, Time);
    RangeList.add(OFFSET_STATE, Time);

    /** add constant value model*/
    if (Time > TimeFirst)
    {
      ConstValList.add(OFFSET_STATE, TimeOld);
      ConstValList.add(OFFSET_STATE, Time);
      SimpleGraph.addFactor<libRSF::FactorType::ConstVal1>(ConstValList, NoiseModelCCE);
      ConstValList.clear();
    }

    /** loop over measurements */
    for (int nMeasurement = RangeMeasurements.countElement(PSEUDORANGE_MEASUREMENT, Time); nMeasurement > 0; nMeasurement--)
    {
      SimpleGraph.addFactor<libRSF::FactorType::Pseudorange2>(RangeList, RangeMeasurements.getElement(PSEUDORANGE_MEASUREMENT, Time, nMeasurement-1), NoiseModelRange);
    }
    RangeList.clear();

    TimeOld = Time;
  }
  while(RangeMeasurements.getTimeNext(PSEUDORANGE_MEASUREMENT, Time, Time));

  /** output initialization */
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(OFFSET_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(OFFSET_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(OFFSET_STATE, 2.0).getNameValueString() << std::endl <<std::endl;

  /** solve graph */
  SimpleGraph.solve(SolverOptions);
  SimpleGraph.printReport();

  /** calculate covariance */
  SimpleGraph.computeCovariance(POSITION_STATE);

  /** output result */
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(OFFSET_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(OFFSET_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(OFFSET_STATE, 2.0).getNameValueString() << std::endl;

  /** load expected results - point2*/
  libRSF::SensorDataSet Expected;
  Expected.addElement(libRSF::Data("point2 0.0 0.0 0.0 0.005 0.0 0.0 0.005"));
  Expected.addElement(libRSF::Data("point2 1.0 1.0 0.0075 0.005 0.0 0.0 0.005"));
  Expected.addElement(libRSF::Data("point2 2.0 1.0 0.931271 0.005 0.0 0.0 0.005"));

  /** calculate maximum component-wise absolute difference between solution and expected (mean and covariance) - point2*/
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

  EXPECT_LT(maxAbsErrorMean,0.1);
  EXPECT_LT(maxAbsErrorCov,1e-3);
}

/** main provided by linking to gtest_main */
