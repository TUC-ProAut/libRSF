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
* @file Example_FG_Range.cpp
* @author Tim Pfeifer
* @date 27.07.2016
* @brief An example application to demonstrate the construction of a simple factor graph with range measurements.
* @copyright GNU Public License.
*
*/

#include "Example_FG_Range.h"

void CreateData (libRSF::SensorDataSet &RangeMeasurements)
{
  std::default_random_engine Generator;
  std::normal_distribution<double> Distribution(0.0, STDDEV_RANGE);

  libRSF::Vector1 Range, StdDev, SatID;
  vector<libRSF::Vector2> SatPositions;
  vector<libRSF::Vector2> EgoPositions;

  StdDev << STDDEV_RANGE;

  SatPositions.push_back((libRSF::Vector2() << 10, 10).finished());
  SatPositions.push_back((libRSF::Vector2() << 10, -10).finished());
  SatPositions.push_back((libRSF::Vector2() << -10, 10).finished());
  SatPositions.push_back((libRSF::Vector2() << -10, -10).finished());

  EgoPositions.push_back((libRSF::Vector2() << 0, 0).finished());
  EgoPositions.push_back((libRSF::Vector2() << 1, 0).finished());
  EgoPositions.push_back((libRSF::Vector2() << 1, 1).finished());

  for (int i = 0; i < static_cast<int>(EgoPositions.size()); i++)
  {
    for (int j = 0; j < static_cast<int>(SatPositions.size()); j++)
    {
      Range[0] = (SatPositions.at(j) - EgoPositions.at(i)).norm() + Distribution(Generator);
      SatID << j;

      libRSF::Data RangeMeasurement(libRSF::DataType::Range2, i);
      RangeMeasurement.setMean(Range);
      RangeMeasurement.setStdDevDiagonal(StdDev);
      RangeMeasurement.setValue(libRSF::DataElement::SatPos, SatPositions.at(j));
      RangeMeasurement.setValue(libRSF::DataElement::SatID, SatID);

      RangeMeasurements.addElement(RangeMeasurement);
    }
  }
}

#ifndef TESTMODE // only compile main if not used in test context

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  libRSF::Vector1 StdDev;
  StdDev << STDDEV_RANGE;

  ceres::Solver::Options SolverOptions;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.minimizer_progress_to_stdout = true;

  libRSF::FactorGraph SimpleGraph;
  libRSF::SensorDataSet RangeMeasurements;
  libRSF::StateList PositionList;
  libRSF::GaussianDiagonal<1> NoiseModel;
  NoiseModel.setStdDevDiagonal(StdDev);

  /** construct a set of range Measurements */
  CreateData(RangeMeasurements);

  /** add position variables to graph */
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 0);
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 1);
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 2);

  /** loop over timestamps */
  double Time = 0.0;
  RangeMeasurements.getTimeFirst(RANGE_MEASUREMENT, Time);
  do
  {
    PositionList.add(POSITION_STATE, Time);
    /** loop over measurements */
    for (int nMeasurement = RangeMeasurements.countElement(RANGE_MEASUREMENT, Time); nMeasurement > 0; nMeasurement--)
    {
      SimpleGraph.addFactor<libRSF::FactorType::Range2>(PositionList, RangeMeasurements.getElement(RANGE_MEASUREMENT, Time, nMeasurement-1), NoiseModel);
    }
    PositionList.clear();
  }
  while(RangeMeasurements.getTimeNext(RANGE_MEASUREMENT, Time, Time));

  /** output initialization */
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;

  /** solve graph */
  SimpleGraph.solve(SolverOptions);
  SimpleGraph.printReport();

  /** output result */
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 1.0).getNameValueString() << std::endl;
  std::cout << SimpleGraph.getStateData().getElement(POSITION_STATE, 2.0).getNameValueString() << std::endl;


  return 0;
}

#endif // TESTMODE
