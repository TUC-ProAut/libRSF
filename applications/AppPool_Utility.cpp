/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option); any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de);
 ***************************************************************************/

#include "AppPool_Utility.h"



bool GetFirstTimestamp (const libRSF::SensorDataSet &Measurements,
                        const libRSF::FactorGraphConfig &Config,
                        double &Timestamp)
{
  if(Config.Solution.IsAsync)
  {
    /** if async, take first overall timestamp */
    double Time, TimeFirst;

    if(Measurements.getTimeFirst(Measurements.begin()->first, TimeFirst))
    {
      for(const auto &Sensor : Measurements)
      {
        if(Measurements.getTimeFirst(Sensor.first, Time))
        {
          if (Time < TimeFirst)
          {
            TimeFirst = Time;
          }
        }
      }
      Timestamp = TimeFirst;
    }
    else
    {
      PRINT_WARNING("There is no measurement!");
    }
  }
  else
  {
    /** if sync take timestamp from sync sensor*/
    if(!Measurements.getTimeFirst(Config.Solution.SyncSensor, Timestamp))
    {
       PRINT_ERROR("There is no measurement of: ", Config.Solution.SyncSensor);
       return false;
    }
  }

  return true;
}

bool GetLastTimestamp (const libRSF::SensorDataSet &Measurements,
                       const libRSF::FactorGraphConfig &Config,
                       double &Timestamp)
{
  if(Config.Solution.IsAsync)
  {
    /** if async, take last overall timestamp */
    double Time, TimeLast;
    TimeLast = 0;
    Time = 0;
    for(const auto &Sensor : Measurements)
    {
      if(Measurements.getTimeLast(Sensor.first, Time))
      {
        if (Time > TimeLast)
        {
          TimeLast = Time;
        }
      }
    }
    Timestamp = TimeLast;
  }
  else
  {
    /** if sync take timestamp from sync sensor*/
    if(!Measurements.getTimeLast(Config.Solution.SyncSensor, Timestamp))
    {
       PRINT_ERROR("There is no measurement of: ", Config.Solution.SyncSensor);
       return false;
    }
  }

  return true;
}

bool IncrementTime(const libRSF::FactorGraphConfig &Config,
                   const libRSF::SensorDataSet &Measurements,
                   double &TimeOld,
                   double &TimeNew,
                   const double TimeLast)
{
  /** store old timestamp */
  TimeOld = TimeNew;

  /** check for time limit */
  if(TimeOld >= TimeLast)
  {
    return false;
  }

  if(Config.Solution.IsAsync)
  {
    /** simply increment with predefined rate */
    TimeNew = TimeOld + 1.0/Config.Solution.AsyncRate;
    return true;
  }

      /** increment to timestamp of next measurement */
    return Measurements.getTimeNext(Config.Solution.SyncSensor, TimeOld, TimeNew);

}


void Solve(libRSF::FactorGraph &Graph,
           const libRSF::FactorGraphConfig &Config,
           libRSF::Data & IterationSummary,
           const bool ForceSolve)
{
  const double TimeNow = IterationSummary.getTimestamp();

  switch(Config.Solution.Type)
  {
    case libRSF::SolutionType::Batch:
      {
        if(ForceSolve)
        {
          /** initial solution without robust models */
          if(Config.Ranging.IsActive)
          {
            Graph.disableErrorModel(Config.Ranging.Type);
          }
          if(Config.GNSS.IsActive)
          {
            Graph.disableErrorModel(Config.GNSS.Type);
          }
          Graph.solve(Config.SolverConfig);

          /** exact solution */
          Graph.enableErrorModels();
          Graph.solve(Config.SolverConfig);

          /** adapt error model */
          int n = 0;
          do
          {
            AdaptErrorModel(Graph, Config, IterationSummary);
            Graph.solve(Config.SolverConfig);
            n++;
          }
          while (Graph.getSolverSummary().num_successful_steps > 2 && n < 10);

          /** compute covariance */
          if(Config.Solution.EstimateCov)
          {
            Graph.computeCovariance(POSITION_STATE);

            if(Graph.getStateData().checkID(ORIENTATION_STATE))
            {
              Graph.computeCovariance(ORIENTATION_STATE);
            }
          if(Graph.getStateData().checkID(ANGLE_STATE))
          {
            Graph.computeCovariance(ANGLE_STATE);
          }
          }
        }
      }
      break;

    case libRSF::SolutionType::Window:
      {
        if(Config.GNSS.IsActive || Config.LoopClosure.IsActive || Config.Ranging.IsActive)
        {
          /** for GNSS, we have RANSAC to get a initial solution */
          AdaptErrorModel(Graph, Config, IterationSummary);
          Graph.solve(Config.SolverConfig);
        }
        else
        {
          /** without GNSS, we solve first before we adapt the error model */
          Graph.solve(Config.SolverConfig);
          if(AdaptErrorModel(Graph, Config, IterationSummary))
          {
            Graph.solve(Config.SolverConfig);
          }
        }

        /** compute covariance */
        if(Config.Solution.EstimateCov)
        {
          Graph.computeCovariance(POSITION_STATE, TimeNow);

          if(Graph.getStateData().checkID(ORIENTATION_STATE))
          {
            Graph.computeCovariance(ORIENTATION_STATE, TimeNow);
          }
          if(Graph.getStateData().checkID(ANGLE_STATE))
          {
            Graph.computeCovariance(ANGLE_STATE, TimeNow);
          }
        }

        /** reduce size */
        if (Config.Solution.Marginalize)
        {
          Graph.marginalizeAllStatesOutsideWindow(Config.Solution.WindowLength, TimeNow, 1.01);
        }
        else
        {
          Graph.removeAllStatesOutsideWindow(Config.Solution.WindowLength, TimeNow);
        }
      }
      break;

    case libRSF::SolutionType::Smoother:
      {
        /** adapt error model at first */
        AdaptErrorModel(Graph, Config, IterationSummary);

        /** solve graph afterwards */
        Graph.solve(Config.SolverConfig);

        if(ForceSolve && Config.Solution.EstimateCov)
        {
          /** compute covariance */
          Graph.computeCovariance(POSITION_STATE);
          if(Graph.getStateData().checkID(ORIENTATION_STATE))
          {
            Graph.computeCovariance(ORIENTATION_STATE);
          }
        }
      }
      break;

    case libRSF::SolutionType::SmootherRT:
      {
        /** adapt error model at first */
        AdaptErrorModel(Graph, Config, IterationSummary);

        /** solve graph afterwards */
        Graph.solve(Config.SolverConfig);

        if(Config.Solution.EstimateCov)
        {
          /** Covariance estimation with fixed states can lead to overconfident results? */
          /** Our experience: No, not if the sliding window is long enough! */
//          Graph.setAllVariable();

          /** compute covariance */
          Graph.computeCovariance(POSITION_STATE, TimeNow);
          if(Graph.getStateData().checkID(ORIENTATION_STATE))
          {
            Graph.computeCovariance(ORIENTATION_STATE, TimeNow);
          }
        }

        Graph.setAllConstantOutsideWindow(Config.Solution.WindowLength, TimeNow);
      }
      break;

    case libRSF::SolutionType::Filter:
      {
        /** adapt error model at first */
        AdaptErrorModel(Graph, Config, IterationSummary);

        /** solve with one step */
        Graph.solve(Config.SolverConfig);

        /** compute covariance */
        if(Config.Solution.EstimateCov)
        {
          Graph.computeCovariance(POSITION_STATE, TimeNow);

          if(Graph.getStateData().checkID(ORIENTATION_STATE))
          {
            Graph.computeCovariance(ORIENTATION_STATE, TimeNow);
          }
          if(Graph.getStateData().checkID(ANGLE_STATE))
          {
            Graph.computeCovariance(ANGLE_STATE, TimeNow);
          }
        }

      }
      break;

    case libRSF::SolutionType::None:
      /** do not solve */
      break;

    default:
      PRINT_ERROR("Unknown solution type: ", Config.Solution.Type);
      break;
  }
}

void Save(libRSF::FactorGraph &Graph,
          const libRSF::FactorGraphConfig &Config,
          libRSF::Data & IterationSummary,
          libRSF::StateDataSet &Result,
          const bool SaveFinal)
{
  const double TimeNow = IterationSummary.getTimestamp();

  switch(Config.Solution.Type)
  {
    case libRSF::SolutionType::Batch:
      if (SaveFinal)
      {
        /** save state data */
        Result = Graph.getStateData();

        /** save runtime information */
        IterationSummary.setValueScalar(libRSF::DataElement::DurationSolver, Graph.getSolverDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationCovariance, Graph.getCovarianceDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::IterationSolver, Graph.getSolverIterationsAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationMarginal, Graph.getMarginalDurationAndReset());
        Result.addElement(SOLVE_TIME_STATE, IterationSummary);
      }
      break;

    case libRSF::SolutionType::Smoother:
      if (SaveFinal)
      {
        Result.merge(Graph.getStateData());
      }
      if (!SaveFinal)
      {
        /** save runtime information */
        IterationSummary.setValueScalar(libRSF::DataElement::DurationSolver, Graph.getSolverDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationCovariance, Graph.getCovarianceDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::IterationSolver, Graph.getSolverIterationsAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationMarginal, Graph.getMarginalDurationAndReset());
        Result.addElement(SOLVE_TIME_STATE, IterationSummary);
      }
    break;

    case libRSF::SolutionType::SmootherRT:
    case libRSF::SolutionType::Window:
      if (!SaveFinal)
      {
        Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, TimeNow, 0));

        /** 3DOF Quaternion */
        if(Graph.getStateData().checkElement(ORIENTATION_STATE, TimeNow, 0))
        {
          Result.addElement(ORIENTATION_STATE, Graph.getStateData().getElement(ORIENTATION_STATE, TimeNow, 0));
        }
        /** 1DOF angle */
        if(Graph.getStateData().checkElement(ANGLE_STATE, TimeNow, 0))
        {
          Result.addElement(ANGLE_STATE, Graph.getStateData().getElement(ANGLE_STATE, TimeNow, 0));
        }
        /** IMU Speed+Bias */
        if(Graph.getStateData().checkElement(IMU_STATE, TimeNow, 0))
        {
          Result.addElement(IMU_STATE, Graph.getStateData().getElement(IMU_STATE, TimeNow, 0));
        }

        /** save run time information */
        IterationSummary.setValueScalar(libRSF::DataElement::DurationSolver, Graph.getSolverDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationCovariance, Graph.getCovarianceDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::IterationSolver, Graph.getSolverIterationsAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationMarginal, Graph.getMarginalDurationAndReset());
        Result.addElement(SOLVE_TIME_STATE, IterationSummary);
      }
      break;

    case libRSF::SolutionType::Filter:
      if (!SaveFinal)
      {
        Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, TimeNow, 0));

        /** 3DOF Quaternion */
        if(Graph.getStateData().checkElement(ORIENTATION_STATE, TimeNow, 0))
        {
          Result.addElement(ORIENTATION_STATE, Graph.getStateData().getElement(ORIENTATION_STATE, TimeNow, 0));
        }
        /** 1DOF angle */
        if(Graph.getStateData().checkElement(ANGLE_STATE, TimeNow, 0))
        {
          Result.addElement(ANGLE_STATE, Graph.getStateData().getElement(ANGLE_STATE, TimeNow, 0));
        }

        /** save run time information */
        IterationSummary.setValueScalar(libRSF::DataElement::DurationSolver, Graph.getSolverDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationCovariance, Graph.getCovarianceDurationAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::IterationSolver, Graph.getSolverIterationsAndReset());
        IterationSummary.setValueScalar(libRSF::DataElement::DurationMarginal, Graph.getMarginalDurationAndReset());
        Result.addElement(SOLVE_TIME_STATE, IterationSummary);
      }
      break;

    case libRSF::SolutionType::None:
      if (SaveFinal)
      {
        Result = Graph.getStateData();

        /** save zero solution information */
        IterationSummary.setValue(libRSF::DataElement::DurationSolver, libRSF::Vector1::Zero());
        IterationSummary.setValue(libRSF::DataElement::IterationSolver, libRSF::Vector1::Zero());
        IterationSummary.setValue(libRSF::DataElement::DurationMarginal, libRSF::Vector1::Zero());
        Result.addElement(SOLVE_TIME_STATE, IterationSummary);
      }
      break;

    default:
      PRINT_ERROR("Unknown solution type!");
      break;
  }
}
