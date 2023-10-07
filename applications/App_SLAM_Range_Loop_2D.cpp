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

#include "App_SLAM_Range_Loop_2D.h"

int main(int ArgC, char** ArgV)
{
  google::InitGoogleLogging(*ArgV);

  /** process command line parameter */
  libRSF::FactorGraphConfig Config;
  if (!Config.ReadCommandLineOptions(ArgC, ArgV))
  {
    PRINT_ERROR("Reading configuration gone wrong! Exit now!");
    return 1;
  }

  /** read input data */
  libRSF::SensorDataSet Measurements;
  libRSF::ReadDataFromFile(Config.InputFile, Measurements);

  /** define first time stamp */
  if (Measurements.empty())
  {
    PRINT_ERROR("Dataset is empty!");
    return 1;
  }

  /** check which sensor data is available */
  Config.LoopClosure.IsActive = (Config.LoopClosure.IsActive &&  Measurements.checkID(libRSF::DataType::LoopClosure));
  Config.Ranging.IsActive = (Config.Ranging.IsActive &&  Measurements.checkID(libRSF::DataType::RangeLM2));

  /** get first and last timestamp */
  double TimeFirst = 0.0;
  double TimeLast = 0.0;
  Measurements.getTimeFirst(libRSF::DataType::Odom2, TimeFirst);
  TimeFirst -= 1.0;
  Measurements.getTimeLast(libRSF::DataType::Odom2, TimeLast);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;

  /** create prior noise models */
  libRSF::GaussianDiagonal<2> NoisePriorPoint;
  libRSF::GaussianDiagonal<1> NoisePriorAngle;
  NoisePriorPoint.setStdDevSharedDiagonal(0.1);
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
  bool KeepRunning = true;
  bool FirstIteration = true;
  while (KeepRunning)
  {
    /** update current timestamp and reset durations */
    Summary = libRSF::Data(libRSF::DataType::IterationSummary, TimeNow);

    /** start timer*/
    IterationTimer.reset();

    /** add new states and odometry */
    if (!FirstIteration)
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

    if (Config.Ranging.IsActive)
    {
      double TimeRangeLow = TimeNow;
      Measurements.getTimeAbove(libRSF::DataType::RangeLM2, TimePrev, TimeRangeLow);
      double TimeRangeHigh = TimeNow;
      Measurements.getTimeBelowOrEqual(libRSF::DataType::RangeLM2, TimeNow, TimeRangeHigh);

      /** get range measurements, if available in window */
      if (TimeRangeLow <= TimeRangeHigh)
      {
        const std::vector<libRSF::Data> Ranges = Measurements.getElementsBetween(libRSF::DataType::RangeLM2, TimeRangeLow, TimeRangeHigh);

        /** add range measurements */
        for (const libRSF::Data &Range : Ranges)
        {
          AddRangeToPoint2(Graph, Config, Range, TimeNow);
        }
      }
      else
      {
        PRINT_WARNING("No range measurements available at timestamp: ", TimeNow);
      }
    }

    if (Config.LoopClosure.IsActive)
    {
      AddLoopClosures(Graph, Config, Measurements, TimePrev, TimeNow);
    }

    /** adapt error model */
    AdaptErrorModel(Graph, Config, Summary);

    /** solve problem */
    Solve(Graph, Config, Summary, false);

    /** save iteration timestamp */
    Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());

    /** save current estimate */
    Save(Graph, Config, Summary, Result, false);

    /** save last timestamp */
    TimePrev = TimeNow;

    /** get next timestamp */
    if (FirstIteration)
    {
      FirstIteration = false;
      KeepRunning = Measurements.getTimeFirst(libRSF::DataType::Odom2, TimeNow);
    }
    else
    {
      KeepRunning = Measurements.getTimeNext(libRSF::DataType::Odom2, TimePrev, TimeNow);
    }

    /** print progress every 10%*/
    libRSF::PrintProgress((TimeNow - TimeFirst) / (TimeLast - TimeFirst) * 100);
  }

  /** calculate and save final solution*/
  Summary.setTimestamp(TimeNow);
  Solve(Graph, Config, Summary, true);
  Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());
  Save(Graph, Config, Summary, Result, true);

  /** get all IDs */
  std::vector<std::string> Keys = Result.getKeysAll();
  /** make unique */
  std::sort(Keys.begin(), Keys.end());
  Keys.erase(std::unique(Keys.begin(), Keys.end()), Keys.end());

  /** loop over entries and export all to file */
  bool FirstEntry = true;
  for (const string& Key : Keys)
  {
    libRSF::WriteDataToFile(Config.OutputFile, Key, Result, !FirstEntry);
    FirstEntry = false;
  }

  return 0;
}
