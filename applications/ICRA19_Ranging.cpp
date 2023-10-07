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
 * @file ICRA19_Ranging.cpp
 * @author Tim Pfeifer
 * @date 17 Sep 2018
 * @brief File containing an application for 2D pose estimation based on range measurements. This special version is made for ICRA 2019.
 * @copyright GNU Public License.
 *
 */

#include "ICRA19_Ranging.h"

/** @brief Build the factor Graph with initial values and a first set of measurements
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the range measurements
 * @param Config reference to the factor graph config
 * @param Options solver option to estimate initial values
 * @param TimestampFirst first timestamp in the Dataset
 * @return nothing
 *
 */
void InitGraph(libRSF::FactorGraph &Graph,
               libRSF::SensorDataSet &Measurements,
               libRSF::FactorGraphConfig const &Config,
               ceres::Solver::Options Options,
               double TimestampFirst)
{
  /** build simple graph */
  libRSF::FactorGraph SimpleGraph;
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, TimestampFirst);

  /** add multiple range measurements to get useful initialization */
  double Timestamp;
  libRSF::StateList ListRange;
  libRSF::Data Range;
  libRSF::GaussianDiagonal<1> NoiseRange;

  ListRange.add(POSITION_STATE, TimestampFirst);
  Timestamp = TimestampFirst;

  for(int i = 0; i < 4; ++i)
  {
    Range = Measurements.getElement(libRSF::DataType::Range2, Timestamp);
    NoiseRange.setStdDevDiagonal(Range.getStdDevDiagonal());
    SimpleGraph.addFactor<libRSF::FactorType::Range2>(ListRange, Range, NoiseRange);

    Measurements.getTimeNext(libRSF::DataType::Range2, Timestamp, Timestamp);
  }

  /** solve */
  Options.minimizer_progress_to_stdout = false;

  SimpleGraph.solve(Options);

  /**add first state variables */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimestampFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimestampFirst);

  /** copy values to real graph */
  Graph.getStateData().getElement(POSITION_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(POSITION_STATE, TimestampFirst).getMean());

  /** add fist set of measurements */
  AddRangeMeasurements2D(Graph, Measurements, Config, TimestampFirst);
}


/** @brief Adds a 2D range measurement to the graph
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the measurement
 * @param Config reference to the factor graph config object that specifies the motion model
 * @param Timestamp a double timestamp of the current position
 * @return nothing
 *
 */
void AddRangeMeasurements2D(libRSF::FactorGraph &Graph,
                            libRSF::SensorDataSet &Measurements,
                            libRSF::FactorGraphConfig const &Config,
                            double Timestamp)
{
  libRSF::StateList ListRange;
  libRSF::Data Range;
  libRSF::GaussianDiagonal<1> NoiseRange;

  ListRange.add(POSITION_STATE, Timestamp);

  /** default error model */
  static libRSF::GaussianMixture<1> GMM((libRSF::Vector2() << 0, 0).finished(),
                                         (libRSF::Vector2() << 0.1, 1.0).finished(),
                                         (libRSF::Vector2() << 0.5, 0.5).finished());

  /** get measurement */
  Range = Measurements.getElement(libRSF::DataType::Range2, Timestamp, 0);

  /** add factor */
  switch(Config.Ranging.ErrorModel.Type)
  {
    case libRSF::ErrorModelType::Gaussian:
      NoiseRange.setStdDevDiagonal(Range.getStdDevDiagonal());
      Graph.addFactor<libRSF::FactorType::Range2>(ListRange, Range, NoiseRange);
      break;

    case libRSF::ErrorModelType::DCS:
      NoiseRange.setStdDevDiagonal(Range.getStdDevDiagonal());
      Graph.addFactor<libRSF::FactorType::Range2>(ListRange, Range, NoiseRange, new libRSF::DCSLoss(1.0));
      break;

    case libRSF::ErrorModelType::cDCE:
      NoiseRange.setStdDevDiagonal(libRSF::Matrix11::Ones());
      Graph.addFactor<libRSF::FactorType::Range2>(ListRange, Range, NoiseRange, new libRSF::cDCELoss(Range.getStdDevDiagonal()[0]));
      break;

    case libRSF::ErrorModelType::GMM:
      if (Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
      {
        static libRSF::MaxMix1 NoiseRangeMaxMix(GMM);
        Graph.addFactor<libRSF::FactorType::Range2>(ListRange, Range, NoiseRangeMaxMix);
      }
      else if (Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
      {
        static libRSF::SumMix1 NoiseRangeSumMix(GMM);
        Graph.addFactor<libRSF::FactorType::Range2>(ListRange, Range, NoiseRangeSumMix);
      }
      else
      {
        PRINT_ERROR("Wrong error model mixture type!");
      }
      break;


    default:
      PRINT_ERROR("Wrong error model type: ", Config.Ranging.ErrorModel.Type);
      break;
  }

}

/** @brief Use a GMM to estimate the error distribution of a factor
 *
 * @param Graph reference to the factor graph object
 * @param Config reference to the factor graph config object that specifies the motion model
 * @param NumberOfComponents how many Gaussian components should be used
 * @param Timestamp current timestamp
 * @return nothing
 *
 */
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config,
                    int NumberOfComponents)
{
  libRSF::GaussianMixture<1> GMM;
  if(Config.Ranging.ErrorModel.GMM.TuningType == libRSF::ErrorModelTuningType::EM)
  {
    /** fill empty GMM */
    if(GMM.getNumberOfComponents() == 0)
    {
      libRSF::GaussianComponent<1> Component;

      for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
      {

        Component.setParamsStdDev((libRSF::Vector1() << 0.1).finished()*pow(10, nComponent),
                                  (libRSF::Vector1() << 0.0).finished(),
                                  (libRSF::Vector1() << 1.0/NumberOfComponents).finished());

        GMM.addComponent(Component);
      }
    }

    libRSF::Matrix ErrorData;
    Graph.computeUnweightedErrorMatrix(libRSF::FactorType::Range2, ErrorData);

    /** call the EM algorithm */
    libRSF::GaussianMixture<1>::EstimationConfig GMMConfig;
    GMMConfig.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM;
    GMM.estimate(ErrorData, GMMConfig);

    /** apply error model */
    if(Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
    {
      libRSF::SumMix1 NewSMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Range2, NewSMModel);
    }
    else if(Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
    {
      libRSF::MaxMix1 NewMMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Range2, NewMMModel);
    }
  }
}

bool ParseErrorModel(const std::string &ErrorModel, libRSF::FactorGraphConfig &Config)
{
  if(ErrorModel == "gauss")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "dcs")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::DCS;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "cdce")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::cDCE;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "sm")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.Ranging.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "mm")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.Ranging.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "stsm")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.Ranging.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::EM;
  }
  else if(ErrorModel == "stmm")
  {
    Config.Ranging.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.Ranging.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::EM;
  }
  else
  {
    PRINT_ERROR("Wrong Error Model: ", ErrorModel);
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(*argv);
  libRSF::FactorGraphConfig Config;

  /** assign all arguments to string vector*/
  std::vector<std::string> Arguments;
  Arguments.assign(argv+1, argv + argc);

  /** read filenames */
  Config.InputFile = Arguments.at(0);
  Config.OutputFile = Arguments.at(1);

  /** parse the error model string */
  if (!ParseErrorModel(Arguments.at(3), Config))
  {
    return 1;
  }

  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.max_num_iterations = 1000;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.max_solver_time_in_seconds = 0.25;

  const int NumberOfComponents = 2;

  /** read input data */
  libRSF::SensorDataSet InputData;
  libRSF::ReadDataFromFile(Config.InputFile, InputData);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;
  libRSF::Data DeltaTime;

  double Timestamp = 0.0, TimestampFirst = 0.0, TimestampOld = 0.0, TimestampLast = 0.0;
  InputData.getTimeFirst(libRSF::DataType::Range2, TimestampFirst);
  InputData.getTimeLast(libRSF::DataType::Range2, TimestampLast);
  Timestamp = TimestampFirst;
  TimestampOld = TimestampFirst;
  int nTimestamp = 0;

  /** add fist variables and factors */
  InitGraph(Graph, InputData, Config, SolverOptions, TimestampFirst);

  /** solve factor graph*/
  Graph.solve(SolverOptions);

  /** safe result at first timestamp */
  Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Timestamp, 0));

  /** get odometry noise from first measurement */
  libRSF::Data Odom = InputData.getElement(libRSF::DataType::Odom2Diff, Timestamp);
  libRSF::GaussianDiagonal<3> NoiseOdom2Diff;
  NoiseOdom2Diff.setStdDevDiagonal(Odom.getStdDevDiagonal());

  /** iterate over timestamps */
  while(InputData.getTimeNext(libRSF::DataType::Range2, Timestamp, Timestamp))
  {

    /** add required states */
    Graph.addState(POSITION_STATE, libRSF::DataType::Point2, Timestamp);
    Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, Timestamp);

    /** add motion model or odometry */
    libRSF::StateList MotionList;
    MotionList.add(POSITION_STATE, TimestampOld);
    MotionList.add(ORIENTATION_STATE, TimestampOld);
    MotionList.add(POSITION_STATE, Timestamp);
    MotionList.add(ORIENTATION_STATE, Timestamp);
    Graph.addFactor<libRSF::FactorType::Odom2Diff>(MotionList, InputData.getElement(libRSF::DataType::Odom2Diff, Timestamp), NoiseOdom2Diff);

    /** add all range measurements of with current timestamp */
    AddRangeMeasurements2D(Graph, InputData, Config, Timestamp);

    /** tune self-tuning error model */
    TuneErrorModel(Graph, Config, NumberOfComponents);

    /** solve factor graph*/
    Graph.solve(SolverOptions);

    /** save data after optimization */
    Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Timestamp, 0));

    /** apply sliding window */
    Graph.removeAllFactorsOutsideWindow(60, Timestamp);
    Graph.removeAllStatesOutsideWindow(60, Timestamp);

    /** save time stamp */
    TimestampOld = Timestamp;

    nTimestamp++;
  }

  /** print last report */
  Graph.printReport();

  /** write results to disk */
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result);

  return 0;
}
