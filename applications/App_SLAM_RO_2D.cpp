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

#include <cmath>

#include "App_SLAM_RO_2D.h"


int main(int ArgC, char** ArgV)
{
  google::InitGoogleLogging(*ArgV);

  /** process command line parameter */
  libRSF::FactorGraphConfig Config;
  if (!Config.ReadCommandLineOptions(ArgC, ArgV))
  {
    PRINT_ERROR("Reading configuration gone wrong! Exit now!");
    return 0;
  }

  /** read input data */
  libRSF::SensorDataSet Measurements;
  libRSF::ReadDataFromFile(Config.InputFile, Measurements);

  /** find first time stamp */
  double TimeFirst = NAN;
  if (!Measurements.getTimeFirstOverall(TimeFirst))
  {
    PRINT_ERROR("Dataset is empty!");
    return 1;
  }

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;

  /** create prior noise models */
  libRSF::GaussianDiagonal<2> NoisePriorPoint;
  NoisePriorPoint.setStdDevSharedDiagonal(0.1);

  libRSF::GaussianDiagonal<1> NoisePriorAngle;
  NoisePriorAngle.setStdDevSharedDiagonal(0.01);

  /** create prior measurements */
  libRSF::Data PriorPoint(libRSF::DataType::Point2, TimeFirst);
  libRSF::Data PriorAngle(libRSF::DataType::Angle, TimeFirst);
  PriorPoint.setMean(libRSF::Vector2::Zero());
  PriorAngle.setMean(libRSF::Vector1::Zero());

  /** add first states */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeFirst);

  /** set constant*/
  Graph.setConstant(POSITION_STATE, TimeFirst);
  Graph.setConstant(ORIENTATION_STATE, TimeFirst);

  /** add prior */
  Graph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, TimeFirst), PriorPoint, NoisePriorPoint);
  Graph.addFactor<libRSF::FactorType::PriorAngle>(libRSF::StateID(ORIENTATION_STATE, TimeFirst), PriorAngle, NoisePriorAngle);

  /** duration of different steps */
  libRSF::Data Summary(libRSF::DataType::IterationSummary, 0.0);
  libRSF::Timer IterationTimer;

  /** loop over measurements */
  double TimeNow = TimeFirst;
  double TimePrev = TimeFirst;
  do
  {
    /** update current timestamp and reset durations */
    Summary = libRSF::Data(libRSF::DataType::IterationSummary, TimeNow);

    /** start timer*/
    IterationTimer.reset();

    /** add new states and odometry */
    if (TimeNow != TimeFirst)
    {
      /** new states */
      Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeNow);
      Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeNow);

      /** get odometry measurement */
      double TimeOdom = 0.0;
      if (!Measurements.getTimeBelowOrEqual(libRSF::DataType::Odom2, TimeNow, TimeOdom))
      {
        PRINT_ERROR("Could not find measurement below: ", TimeNow);
        return 0;
      }
      const libRSF::Data Odom = Measurements.getElement(libRSF::DataType::Odom2, TimeOdom);

      /** create noise model */
      libRSF::GaussianDiagonal<3> GaussianOdom;
      GaussianOdom.setCovarianceDiagonal(Odom.getCovarianceDiagonal());

      /** add odometry factor */
      Graph.addFactor<libRSF::FactorType::Odom2>(libRSF::StateID(POSITION_STATE, TimePrev),
                                                 libRSF::StateID(ORIENTATION_STATE, TimePrev),
                                                 libRSF::StateID(POSITION_STATE, TimeNow),
                                                 libRSF::StateID(ORIENTATION_STATE, TimeNow),
                                                 Odom, GaussianOdom);
    }

    /** get range measurements */
    const std::vector<libRSF::Data> Ranges = Measurements.getElements(libRSF::DataType::RangeLM2, TimeNow);

    /** add range measurements */
    for (const libRSF::Data &Range: Ranges)
    {
      AddRangeToPoint2(Graph, Config, Range, TimeNow);
    }

    /** solve */
//    Solve(Graph, Config, Summary, false);

    /** adapt error model */
    if(AdaptErrorModel(Graph, Config, Summary))
    {
      /** solve again*/
//      Solve(Graph, Config, Summary, false);
    }

    Solve(Graph, Config, Summary, false);

    /** save iteration timestamp */
    Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());

    /** save current estimate */
    Save(Graph, Config, Summary, Result, false);

    TimePrev = TimeNow;
  }
  while (Measurements.getTimeNext(libRSF::DataType::RangeLM2, TimePrev, TimeNow));

  /** calculate and save final solution*/
  Summary.setTimestamp(TimeNow);
  Solve(Graph, Config, Summary, true);
  Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());
  Save(Graph, Config, Summary, Result, true);

  /** export result to file */
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result, false);
  libRSF::WriteDataToFile(Config.OutputFile, ORIENTATION_STATE, Result, true);
  libRSF::WriteDataToFile(Config.OutputFile, SOLVE_TIME_STATE, Result, true);

  return 0;
}
