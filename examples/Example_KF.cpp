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
* @file Example_KF.cpp
* @author Tim Pfeifer
* @date 25.11.2019
* @brief An example application to demonstrate how a Kalman filter can be implemented with ceres.
* @copyright GNU Public License.
*
*/

#include <ceres/ceres.h>
#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position"
#define RANGE_MEASUREMENT libRSF::DataType::Range2

#define STDDEV_RANGE  0.25
#define STDDEV_CP     1.0

void CreateData (libRSF::SensorDataSet &RangeMeasurements)
{
  std::default_random_engine Generator(42);
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
  EgoPositions.push_back((libRSF::Vector2() << 1, 2).finished());
  EgoPositions.push_back((libRSF::Vector2() << 1, 3).finished());
  EgoPositions.push_back((libRSF::Vector2() << 2, 3).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 3).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 4).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 5).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 6).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 7).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 8).finished());
  EgoPositions.push_back((libRSF::Vector2() << 4, 8).finished());
  EgoPositions.push_back((libRSF::Vector2() << 3, 8).finished());
  EgoPositions.push_back((libRSF::Vector2() << 2, 8).finished());
  EgoPositions.push_back((libRSF::Vector2() << 1, 8).finished());
  EgoPositions.push_back((libRSF::Vector2() << 0, 8).finished());
  EgoPositions.push_back((libRSF::Vector2() << -1, 8).finished());

  for (int i = 0; i < static_cast<int>(EgoPositions.size()); i++)
  {
    for (int j = 0; j < static_cast<int>(SatPositions.size()); j++)
    {
      Range[0] = (SatPositions.at(j) - EgoPositions.at(i)).norm() + Distribution(Generator);
      SatID << j;

      libRSF::Data RangeMeasurement(libRSF::DataType::Range2, i);
      RangeMeasurement.setMean(Range);
      RangeMeasurement.setCovarianceDiagonal(StdDev);
      RangeMeasurement.setValue(libRSF::DataElement::SatPos, SatPositions.at(j));
      RangeMeasurement.setValue(libRSF::DataElement::SatID, SatID);

      RangeMeasurements.addElement(RangeMeasurement);
    }
  }
}

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  double Time = 0.0, TimeOld = 0.0;

  ceres::Solver::Options FGOptions;
  FGOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  FGOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;

  ceres::Solver::Options KFOptions;
  KFOptions.use_nonmonotonic_steps = true;
  KFOptions.linear_solver_type = ceres::DENSE_QR;
  KFOptions.initial_trust_region_radius = 1e16;
  KFOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
  KFOptions.max_num_iterations = 1;

  libRSF::FactorGraph SimpleGraph;
  libRSF::FactorGraph KFGraph;

  /** create error models */
  libRSF::GaussianDiagonal<1> NoiseRange;
  NoiseRange.setStdDevSharedDiagonal(STDDEV_RANGE);

  libRSF::GaussianDiagonal<2> NoiseCP;
  NoiseCP.setStdDevSharedDiagonal(STDDEV_CP);

  /** construct a set of range Measurements */
  libRSF::SensorDataSet RangeMeasurements;
  CreateData(RangeMeasurements);

  /** loop over timestamps */
  RangeMeasurements.getTimeFirst(RANGE_MEASUREMENT, Time);
  do
  {
    /** add position variables to graph */
    SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, Time);
    KFGraph.addState(POSITION_STATE, libRSF::DataType::Point2, Time);

    /** add motion model */
    if(Time > 0)
    {
      SimpleGraph.addFactor<libRSF::FactorType::ConstVal2>(libRSF::StateID(POSITION_STATE, TimeOld), libRSF::StateID(POSITION_STATE, Time), NoiseCP);
      KFGraph.addFactor<libRSF::FactorType::ConstVal2>(libRSF::StateID(POSITION_STATE, TimeOld), libRSF::StateID(POSITION_STATE, Time), NoiseCP);

      /** remove old state of KF */
      KFGraph.marginalizeState(POSITION_STATE, TimeOld, 0);
    }

    /** add range measurements */
    for (int nMeasurement = RangeMeasurements.countElement(RANGE_MEASUREMENT, Time); nMeasurement > 0; nMeasurement--)
    {
      libRSF::Data Range =  RangeMeasurements.getElement(RANGE_MEASUREMENT, Time, nMeasurement-1);
      SimpleGraph.addFactor<libRSF::FactorType::Range2>(libRSF::StateID(POSITION_STATE, Time),Range , NoiseRange);
      KFGraph.addFactor<libRSF::FactorType::Range2>(libRSF::StateID(POSITION_STATE, Time),Range , NoiseRange);
    }

    /** solve KF */
    KFGraph.solve(KFOptions);
    KFGraph.computeCovariance(POSITION_STATE);

    /** solve FGO */
    SimpleGraph.solve(FGOptions);
    SimpleGraph.computeCovariance(POSITION_STATE);

    /** print result */
    std::cout << "KF:  " << KFGraph.getStateData().getElement(POSITION_STATE, Time).getNameValueString() << std::endl;
    std::cout << "FGO: " << SimpleGraph.getStateData().getElement(POSITION_STATE, Time).getNameValueString() << std::endl << std::endl;

    /** save current timestamp */
    TimeOld = Time;
  }
  while(RangeMeasurements.getTimeNext(RANGE_MEASUREMENT, Time, Time));

  SimpleGraph.printReport();

  return 0;
}
