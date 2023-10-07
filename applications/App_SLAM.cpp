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

#include "App_SLAM.h"

void AddLoopClosure2(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     const libRSF::Data &Loop)
{
  /** find closest timestamps */
  double Time1 = std::numeric_limits<double>::quiet_NaN();
  double Time2 = std::numeric_limits<double>::quiet_NaN();
  Time1 = Loop.getTimestamp();
  Time2 = Loop.getValue(libRSF::DataElement::TimestampRef)(0);
  Graph.getStateData().getTimeCloseTo(POSITION_STATE, Time1, Time1);
  Graph.getStateData().getTimeCloseTo(POSITION_STATE, Time2, Time2);

  /** fixed noise */
  const double StdDevLoop = Config.LoopClosure.Parameter(1);

  /** add factor with right error model */
  switch(Config.LoopClosure.ErrorModel.Type)
  {
    case libRSF::ErrorModelType::Gaussian:
      {
        libRSF::GaussianDiagonal<2> Noise;
        Noise.setStdDevSharedDiagonal(StdDevLoop);

        Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                       libRSF::StateID(POSITION_STATE, Time2),
                                                       Noise);
      }
      break;

    case libRSF::ErrorModelType::GMM:
      {
        /** init model */
        libRSF::GaussianMixture<2> const GMM(Config.LoopClosure.ErrorModel.GMM);

        /**Max-Mixture GMM [Olson et al.]*/
        if(Config.LoopClosure.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
        {
          libRSF::MaxMix2 const MixtureNoise(GMM);
          Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                         libRSF::StateID(POSITION_STATE, Time2),
                                                         MixtureNoise);
        }
        /**Sum-Mixture GMM [Rosen et al.]*/
        else if(Config.LoopClosure.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
        {
          libRSF::SumMix2 const MixtureNoise(GMM);
          Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                         libRSF::StateID(POSITION_STATE, Time2),
                                                         MixtureNoise);
        }
        /**Max-Sum-Mix-Mixture GMM [Pfeifer et al.]*/
        else if(Config.LoopClosure.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxSumMix)
        {
          libRSF::MaxSumMix2 const MixtureNoise(GMM);
          Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                         libRSF::StateID(POSITION_STATE, Time2),
                                                         MixtureNoise);
        }
        else
        {
          PRINT_ERROR("Wrong mixture type!");
        }
      }
      break;

    case libRSF::ErrorModelType::SC:
      {
        libRSF::GaussianDiagonal<2> Noise;
        Noise.setStdDevSharedDiagonal(StdDevLoop);

        /**add switch variable */
        Graph.addState(SWITCH_STATE, libRSF::DataType::Switch, Time2);

        /** create Switchable Constraints error model */
        libRSF::SwitchableConstraints<2, libRSF::GaussianDiagonal<2>> const SC(Noise, Config.LoopClosure.ErrorModel.Parameter);


        Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                       libRSF::StateID(POSITION_STATE, Time2),
                                                       libRSF::StateID(SWITCH_STATE, Time2),
                                                       SC);
      }
      break;

    case libRSF::ErrorModelType::DCS:
      {
        libRSF::GaussianDiagonal<2> Noise;
        Noise.setStdDevSharedDiagonal(StdDevLoop);

        Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                       libRSF::StateID(POSITION_STATE, Time2),
                                                       Noise,
                                                       new libRSF::DCSLoss(Config.LoopClosure.ErrorModel.Parameter));
      }
      break;

    case libRSF::ErrorModelType::DCE:
      {

        /**add covariance variable */
        Graph.addState(COV_STATE, libRSF::DataType::Covariance2, Time2);
        const int CovIndex = Graph.getStateData().countElement(COV_STATE, Time2) - 1;
        const libRSF::Vector2 Cov = libRSF::Vector2::Ones()*StdDevLoop*StdDevLoop;

        /** set initial value */
        Graph.getStateData().getElement(COV_STATE, Time2, CovIndex).setMean(Cov);

        /** set lower bound */
        Graph.setLowerBound(COV_STATE, Time2, CovIndex, Cov);

        /** create Dynamic Covariance Estimation error model */
        libRSF::DynamicCovarianceEstimation<2> const DCE(Cov);

        /** add to graph */
        Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                       libRSF::StateID(POSITION_STATE, Time2),
                                                       libRSF::StateID(COV_STATE, Time2),
                                                       DCE);
      }
      break;

    case libRSF::ErrorModelType::cDCE:
      {
        libRSF::GaussianDiagonal<2> Noise;
        Noise.setStdDevSharedDiagonal(1.0);

        Graph.addFactor<libRSF::FactorType::Loop2>(libRSF::StateID(POSITION_STATE, Time1),
                                                       libRSF::StateID(POSITION_STATE, Time2),
                                                       Noise,
                                                       new libRSF::cDCELoss(StdDevLoop));
      }
      break;

    default:
      PRINT_ERROR("Wrong error type: ", Config.LoopClosure.ErrorModel.Type);
      break;
  }
}

int CreateGraphAndSolve(const libRSF::FactorGraphConfig &Config,
                        libRSF::StateDataSet &Result)
{

  /** read input data */
  libRSF::SensorDataSet Measurements;
  libRSF::ReadDataFromFile(Config.InputFile, Measurements);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;

  /** duration of different steps */
  libRSF::Data Summary(libRSF::DataType::IterationSummary, 0.0);

  /** find time boarders */
  double TimeFirst = 0.0, TimeLast = 0.0, TimeNow = 0.0, TimeOld = 0.0;
  Measurements.getTimeFirst(libRSF::DataType::Odom2, TimeFirst);
  Measurements.getTimeLast(libRSF::DataType::Odom2, TimeLast);

  /** set prior vectors */
  const libRSF::Vector1 AngleVect = libRSF::Vector1::Ones() * M_PI;
  const libRSF::Vector2 PositionVect = libRSF::Vector2::Zero();

  /** create prior noise models */
  libRSF::GaussianDiagonal<2> NoisePriorPoint;
  NoisePriorPoint.setStdDevSharedDiagonal(0.1);

  libRSF::GaussianDiagonal<1> NoisePriorAngle;
  NoisePriorAngle.setStdDevSharedDiagonal(0.1);

  /** create prior measurements */
  libRSF::Data PriorPoint(libRSF::DataType::Point2, TimeFirst);
  libRSF::Data PriorAngle(libRSF::DataType::Angle, TimeFirst);
  PriorPoint.setMean(PositionVect);
  PriorAngle.setMean(AngleVect);

  /** add first states */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeFirst);

  /** add prior */
  Graph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, TimeFirst), PriorPoint, NoisePriorPoint);
  Graph.addFactor<libRSF::FactorType::PriorAngle>(libRSF::StateID(ORIENTATION_STATE, TimeFirst), PriorAngle, NoisePriorAngle);

  /** loop over odometry */
  TimeOld = TimeFirst - 1;
  TimeNow = TimeFirst;
  libRSF::Timer IterationTimer;
  do
  {
    /** update current timestamp and reset durations */
    Summary = libRSF::Data(libRSF::DataType::IterationSummary, TimeNow);

    /** start timer*/
    IterationTimer.reset();

    if (TimeNow > TimeFirst)
    {
      /** add states */
      Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeNow);
      Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeNow);

       /** get odometry measurements */
      const std::vector<libRSF::Data> OdomVect = Measurements.getElementsBetween(libRSF::DataType::Odom2, TimeOld, TimeNow);
      const libRSF::Data Odom =  libRSF::AverageMeasurement(OdomVect);

      /** create error model odom */
      libRSF::GaussianDiagonal<3> NoiseOdom;
      NoiseOdom.setCovarianceDiagonal(Odom.getCovarianceDiagonal());

      /** add odometry factor */
      Graph.addFactor<libRSF::FactorType::Odom2>(
                      libRSF::StateID(POSITION_STATE, TimeOld),
                      libRSF::StateID(ORIENTATION_STATE, TimeOld),
                      libRSF::StateID(POSITION_STATE, TimeNow),
                      libRSF::StateID(ORIENTATION_STATE, TimeNow),
                      Odom,
                      NoiseOdom);
    }

    /** search for next loop */
    double TimeNextLoop = TimeLast + 1e10;
    Measurements.getTimeAbove(libRSF::DataType::LoopClosure, TimeOld, TimeNextLoop);

    /** get loops if there is any */
    if (TimeNextLoop <= TimeNow)
    {
      /** get loop-closures between the last and the current timestamp */
      std::vector<libRSF::Data> const LoopVect = Measurements.getElementsBetween(libRSF::DataType::LoopClosure, TimeOld, TimeNow);

      /** add loop-closures */
      static double FirstLoop = TimeLast;
      for(const libRSF::Data &Loop : LoopVect)
      {
          /** add only loops with a similarity above or equal the threshold */
          if (Loop.getValue(libRSF::DataElement::Similarity)(0) >= Config.LoopClosure.Parameter(0))
          {
            FirstLoop = std::min(FirstLoop, std::min(Loop.getTimestamp(), Loop.getValue(libRSF::DataElement::TimestampRef)(0)));
            AddLoopClosure2(Graph, Config, Loop);
          }
      }

      /** optimize the full graph back*/
      Graph.setAllVariableInsideWindow(TimeNow - FirstLoop, TimeNow);
    }

    /** refine error model initially */
    if (TimeNow == TimeFirst)
    {
      Graph.solve(Config.SolverConfig);
      Solve(Graph, Config, Summary, false);
    }
    else
    {
      /** solve graph, force solve every 60 seconds */
      Solve(Graph, Config, Summary, fmod(TimeNow, 60.0) < (TimeNow - TimeOld) * 1.1);
    }

    /** save iteration timestamp */
    Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());

    /** save result */
    Save(Graph, Config, Summary, Result, false);

    /** print progress every 10%*/
    libRSF::PrintProgress((TimeNow - TimeFirst) / (TimeLast - TimeFirst) * 100);
  }
  while (IncrementTime(Config, Measurements, TimeOld, TimeNow, TimeLast));

  /** calculate and save final solution*/
  Summary.setTimestamp(TimeNow);
  Solve(Graph, Config, Summary, true);
  Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());
  Save(Graph, Config, Summary, Result, true);

  /** print last report */
  Graph.printReport();

  return 0;
}

#ifndef TESTMODE // only compile main if not used in test context

int main(int ArgC, char ** ArgV)
{
  google::InitGoogleLogging(*ArgV);

  /** parse command line arguments */
  libRSF::FactorGraphConfig Config;
  Config.ReadCommandLineOptions(ArgC, ArgV);

  /** data structure for estimates*/
  libRSF::StateDataSet Result;

  /** solve the estimation problem */
  if (CreateGraphAndSolve(Config,Result) != 0)
  {
    PRINT_ERROR("Something gone wrong while estimating GNSS position!");
  }
  else
  {
    /** export position estimate to file */
    libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result, false);

    /** additional estimates */
    libRSF::WriteDataToFile(Config.OutputFile, ORIENTATION_STATE, Result, true);

    /** export timing information */
    libRSF::WriteDataToFile(Config.OutputFile, SOLVE_TIME_STATE, Result, true);
  }

  return 0;
}

#endif // TESTMODE
