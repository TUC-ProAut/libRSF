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
 * @file App_multiFusion.cpp
 * @author Tim Pfeifer
 * @date 02 May 2019
 * @brief File containing an universal application for arbitrary factor graphs.
 * @copyright GNU Public License.
 *
 */

#include "AppPool_Adaptive.h"
#include "AppPool_Init.h"
#include "AppPool_Sensors.h"
#include "AppPool_Utility.h"

void Predict(libRSF::FactorGraph &Graph,
             const libRSF::FactorGraphConfig &Config,
             const libRSF::SensorDataSet &Measurements,
             const double TimeOld,
             const double TimeNow)
{
  /** Motion Model */
  if (Config.MotionModel.IsActive)
  {
    AddMotionModel(Graph, Config, TimeOld, TimeNow);
  }

  /** IMU */
  if (Config.IMU.IsActive)
  {
    AddIMU(Graph, Config, Measurements, TimeOld, TimeNow);
  }

  /** Odometry */
  if (Config.Odom.IsActive)
  {
    AddOdometry(Graph, Config.Odom.Type, libRSF::DataType::Odom3, Measurements, TimeOld, TimeNow);
  }

  /** Radar odometry */
  if (Config.Radar.IsActive)
  {
    AddOdometry(Graph, Config.Radar.Type, libRSF::DataType::Odom3Radar, Measurements, TimeOld, TimeNow);
  }

  /** Laser odometry */
  if (Config.Laser.IsActive)
  {
    AddOdometry(Graph, Config.Laser.Type, libRSF::DataType::Odom3Laser, Measurements, TimeOld, TimeNow);
  }

  /** Barometric pressure */
  if (Config.Pressure.IsActive)
  {
    AddPressure(Graph, Config.Pressure.Type, Measurements, TimeOld, TimeNow);
  }

  /** add position state if nobody does this before */
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeNow);

  /** marginalize filter state */
  if (Config.Solution.Type == libRSF::SolutionType::Filter)
  {
    /** marginalize current states */
    Graph.marginalizeAllStatesOutsideWindow((TimeNow - TimeOld) / 2.0, TimeNow, 1.01);
  }
}

void Measure(libRSF::FactorGraph &Graph,
             const libRSF::FactorGraphConfig &Config,
             const libRSF::SensorDataSet &Measurements,
             const double TimeOld,
             const double TimeNow)
{
  /** pseudo range measurements from GNSS */
  if (Config.GNSS.IsActive)
  {
    AddGNSS(Graph, Config, Measurements, TimeOld, TimeNow);
  }

  /** absolute ranging */
  if (Config.Ranging.IsActive)
  {
    AddUWB(Graph, Config, Measurements, TimeOld, TimeNow);
  }

  /** relative loop closures */
  if (Config.LoopClosure.IsActive)
  {
    const bool HasLoop = AddLoopClosures(Graph, Config, Measurements, TimeOld, TimeNow);

    /** in case of loop closures, optimize complete graph */
    if (HasLoop && Config.Solution.Type == libRSF::SolutionType::SmootherRT)
    {
      Graph.setAllVariable();
    }
  }

  /** optional prior for the first position*/
  if (Config.Prior.IsActive && Config.Prior.Type == libRSF::FactorType::Prior3)
  {
    /** prior position */
    libRSF::Data PriorPoint(libRSF::DataType::Point3, TimeNow);

    /** define prior point */
    libRSF::Vector3 Point = Graph.getStateData().getElement(POSITION_STATE, TimeNow).getMean();
    Point(2) = Config.Prior.Parameter(2); /**< height */
    PriorPoint.setMean(Point);

    /** prior uncertainty */
    libRSF::GaussianDiagonal<3> PriorNoise;
    PriorNoise.setSqrtInformationDiagonal(Config.Prior.Parameter.tail(3));

    /** prior factor */
    Graph.addFactor<libRSF::FactorType::Prior3>(libRSF::StateID(POSITION_STATE, TimeNow), PriorPoint, PriorNoise);
  }
}

void InitGraph(libRSF::FactorGraph &Graph,
               libRSF::SensorDataSet &Measurements,
               const libRSF::FactorGraphConfig &Config,
               libRSF::TangentPlaneConverter &LocalFrame,
               const double TimeInitial)
{
  bool IsInitialized = false;

  if (Config.GNSS.IsActive)
  {
    /** use the first GNSS measurements to init the position as well as the tangent plane */
    InitWithGNSS(Graph, Measurements, Config, LocalFrame, TimeInitial, 0.1);
    IsInitialized = true;
  }

  if (Config.Ranging.IsActive)
  {
    /** uses all range measurements in the first 4 second to init the position, use at least 30 */
    InitWithUWB(Graph, Measurements, Config, TimeInitial, 6.0, 30);
    IsInitialized = true;
  }

  if (Config.IMU.IsActive)
  {
    /** uses all IMU measurements in the first 2 seconds to init the IMU biases */
    InitIMU(Graph, Measurements, TimeInitial, 2.0);
  }
  else if (Config.Odom.IsActive)
  {
    /** add generic rotation prior */
    InitOdom(Graph, Config.Odom.Type, TimeInitial);
  }
  else if (Config.Laser.IsActive)
  {
    InitOdom(Graph, Config.Laser.Type, TimeInitial);
  }
  else if (Config.Radar.IsActive)
  {
    InitOdom(Graph, Config.Radar.Type, TimeInitial);
  }

  /** add prior if no initialization was done */
  if (!IsInitialized)
  {
    /** add the first position */
    Graph.addState(POSITION_STATE, libRSF::DataType::Point3, TimeInitial);

    /** add position prior at [0,0,0] */
    libRSF::Data Pos(libRSF::DataType::Point3, TimeInitial);
    Pos.setMean(libRSF::Vector3::Zero());
    libRSF::GaussianDiagonal<3> Gauss;
    Gauss.setStdDevSharedDiagonal(1.0);
    Graph.addFactor<libRSF::FactorType::Prior3>(libRSF::StateID(POSITION_STATE, TimeInitial, 0), Pos, Gauss);

    /** freeze initial states */
    Graph.setAllConstantOutsideWindow(1.0, TimeInitial);
  }
}

int main(int ArgC, char ** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** enable extended logging on debug mode */
#ifndef NDEBUG
  google::LogToStderr();
  google::SetStderrLogging(google::GLOG_INFO);
#endif

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

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;

  /** converter from an earth-centered frame to a local (ENU) frame */
  libRSF::TangentPlaneConverter LocalFrame;

  /** duration of different steps */
  libRSF::Data Summary(libRSF::DataType::IterationSummary, 0.0);

  /** get relevant timestamps */
  double TimeFirst, TimeOld, TimeNow, TimeLast;
  GetFirstTimestamp(Measurements, Config, TimeFirst);

  if (!GetFirstTimestamp(Measurements, Config, TimeFirst))
  {
    PRINT_ERROR("Could not find first Timestamp! Exit now!");
    return 0;
  }
  if (!GetLastTimestamp(Measurements, Config, TimeLast))
  {
    PRINT_ERROR("Could not find last Timestamp! Exit now!");
    return 0;
  }

  /** init factor graph */
  InitGraph(Graph, Measurements, Config, LocalFrame, TimeFirst);

  /** update loop */
  TimeOld = TimeFirst - 1;
  TimeNow = TimeFirst;
  libRSF::Timer IterationTimer;
  do
  {
    /** update current timestamp and reset durations */
    Summary = libRSF::Data(libRSF::DataType::IterationSummary, TimeNow);

    /** start timer*/
    IterationTimer.reset();

    /** predict next state */
    if (TimeNow > TimeFirst)
    {
      Predict(Graph, Config, Measurements, TimeOld, TimeNow);
    }

    /** add measurements */
    Measure(Graph, Config, Measurements, TimeOld, TimeNow);

    /** refine error model initially */
    if (TimeNow == TimeFirst)
    {
      Graph.solve(Config.SolverConfig);
    }
    /** solve graph, force solve every 60 seconds */
    Solve(Graph, Config, Summary, fmod(TimeNow, 60.0) < (TimeNow - TimeOld) * 1.1);

    /** save iteration timestamp */
    Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());

    /** save result */
    Save(Graph, Config, Summary, Result, false);

    /** print progress every 10%*/
    libRSF::PrintProgress((TimeNow - TimeFirst) / (TimeLast - TimeFirst) * 100);
  } while (IncrementTime(Config, Measurements, TimeOld, TimeNow, TimeLast));

  /** calculate and save final solution*/
  Summary.setTimestamp(TimeNow);
  Solve(Graph, Config, Summary, true);
  Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());
  Save(Graph, Config, Summary, Result, true);

  /** convert back in a global frame */
  if (Config.GNSS.IsActive && LocalFrame.isInitialized())
  {
    /** save position in a local coordinate system */
    libRSF::WriteDataToFile(Config.OutputFile + string("_local"), POSITION_STATE, Result);

    /** convert to global frame */
    LocalFrame.convertAllStatesToGlobal(Result, POSITION_STATE);
  }

  /** print last report */
  Graph.printReport();

  /** export result to file */
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result, false);

  /** additional info */
  libRSF::WriteDataToFile(Config.OutputFile, ORIENTATION_STATE, Result, true);
  libRSF::WriteDataToFile(Config.OutputFile, ANGLE_STATE, Result, true);
  libRSF::WriteDataToFile(Config.OutputFile, SOLVE_TIME_STATE, Result, true);
  //  libRSF::WriteDataToFile(Config.OutputFile, IMU_STATE, Result, true);

  /** additional debugging output */
  //  if (Config.GNSS.IsActive)
  //  {
  //    libRSF::StateDataSet Error;
  //    Graph.computeUnweightedError(Config.GNSS.Type, ERROR_STATE, Error);
  //    libRSF::WriteDataToFile(Config.OutputFile, ERROR_STATE, Error, true);
  //  }
  //  if (Config.Ranging.IsActive)
  //  {
  //    libRSF::StateDataSet Error;
  //    Graph.computeUnweightedError(Config.Ranging.Type, ERROR_STATE, Error);
  //    libRSF::WriteDataToFile(Config.OutputFile, ERROR_STATE, Error, true);
  //  }
  //  if (Config.Odom.IsActive)
  //  {
  //    libRSF::StateDataSet Error;
  //    Graph.computeUnweightedError(Config.Odom.Type, ERROR_STATE, Error);
  //    libRSF::WriteDataToFile(Config.OutputFile, ERROR_STATE, Error, true);
  //  }

  return 0;
}
