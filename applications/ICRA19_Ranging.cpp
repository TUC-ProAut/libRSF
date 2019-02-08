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
 * @file ICRA19_Ranging.cpp
 * @author Tim Pfeifer
 * @date 17 Sep 2018
 * @brief File containing an application for 2D pose estimation based on range measurements. This special version is made for ICRA 2019.
 * @copyright GNU Public License.
 *
 */

#include "ICRA19_Ranging.h"


/** @brief Generates a delta time measurement object from two timestamps
 *
 * @param TimestampOld a double timestamp
 * @param TimestampNew another double timestamp
 * @return A SensorData Object, thats represents the time difference
 *
 */
libRSF::SensorData GenerateDeltaTime(const double TimestampOld,
                                      const double TimestampNew)
{
  libRSF::SensorData DeltaTime(libRSF::SensorType::DeltaTime, TimestampNew);
  ceres::Vector DeltaTimeVector(1);
  DeltaTimeVector[0] = TimestampNew - TimestampOld;
  DeltaTime.setMean(DeltaTimeVector);
  DeltaTime.setTimestamp(TimestampNew);

  return DeltaTime;
}

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
  SimpleGraph.addState(POSITION_STATE, libRSF::StateType::Pose2, TimestampFirst);

  /** add multiple range measurements to get useful initialization */
  double Timestamp;
  libRSF::StateList ListRange;
  libRSF::SensorData Range;
  libRSF::GaussianDiagonal<1> NoiseRange;

  ListRange.add(POSITION_STATE, TimestampFirst);
  Timestamp = TimestampFirst;

  for(int i = 0; i < 4; ++i)
  {
    Range = Measurements.getElement(libRSF::Sensors._Config.at(libRSF::SensorType::Range2)._Name, Timestamp);
    NoiseRange.setStdDev(Range.getStdDev());
    SimpleGraph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRange);

    Measurements.getNextTimestamp(libRSF::Sensors._Config.at(libRSF::SensorType::Range2)._Name, Timestamp, Timestamp);
  }

  /** solve */
  Options.minimizer_progress_to_stdout = false;

  SimpleGraph.solve(Options);

  /**add first state variables */
  Graph.addState(POSITION_STATE, libRSF::StateType::Pose2, TimestampFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::StateType::Angle, TimestampFirst);

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
  libRSF::SensorData Range;
  libRSF::GaussianDiagonal<1> NoiseRange;

  ListRange.add(POSITION_STATE, Timestamp);

  /** get measurement */
  Range = Measurements.getElement("Range 2D", Timestamp, 0);

  /** default error model */
  static libRSF::GaussianMixture<1> GMM((ceres::Vector(2) << 0, 0).finished(),
                                         (ceres::Vector(2) << 0.1, 1.0).finished(),
                                         (ceres::Vector(2) << 0.5, 0.5).finished());

  /** add factor */
  switch(Config.RangeErrorModel.Type)
  {
  case ROBUST_NONE:
    NoiseRange.setStdDev(Range.getStdDev());
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRange);
    break;

  case ROBUST_DCS:
    NoiseRange.setStdDev(Range.getStdDev());
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRange, new libRSF::DCSLoss(1.0));
    break;

  case ROBUST_CDCE:
    NoiseRange.setStdDev(Eigen::Matrix<double, 1, 1>::Ones());
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRange, new libRSF::cDCELoss(Range.getStdDev()[0]));
    break;

  case ROBUST_MM:
    static libRSF::MaxMix1 NoiseRangeMaxMix(GMM);
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRangeMaxMix);

    break;

  case ROBUST_SM:
    static libRSF::SumMix1 NoiseRangeSumMix(GMM);
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRangeSumMix);

    break;

  case ROBUST_STSM:
    static libRSF::SumMix1 NoiseRangeSumMixST(GMM);
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRangeSumMixST);

    break;

  case ROBUST_STMM:
    static libRSF::MaxMix1 NoiseRangeMaxMixST(GMM);
    Graph.addFactor(libRSF::FactorType::Range2, ListRange, Range, NoiseRangeMaxMixST);

    break;

  default:
    std::cerr << "Error: Wrong range error 2D model model: " << Config.RangeErrorModel.Type << std::endl;
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
                    int NumberOfComponents,
                    double Timestamp)
{
  if(Config.RangeErrorModel.Type == ROBUST_STSM || Config.RangeErrorModel.Type == ROBUST_STMM)
  {
    std::vector<double> ErrorData;

    libRSF::GaussianMixture<1> GMM;

    /** fill empty GMM */
    if(GMM.getNumberOfComponents() == 0)
    {
      libRSF::GaussianComponent<1> Component;

      for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
      {

        Component.setParamsStdDev((ceres::Vector(1) << 0.1).finished()*pow(10, nComponent),
                                  (ceres::Vector(1) << 0.0).finished(),
                                  (ceres::Vector(1) << 1.0/NumberOfComponents).finished());

        GMM.addComponent(Component);
      }
    }

    Graph.computeUnweightedError(libRSF::FactorType::Range2, ErrorData);

    /** remove zeros in MM results (unused dimension!) */
    if(Config.RangeErrorModel.Type == ROBUST_STMM)
    {
      for(int i = ErrorData.size() - 1; i > 0; i -= 2)
      {
        ErrorData.erase(ErrorData.begin() + i);
      }
    }

    /** call the EM algorithm */
    GMM.estimateWithEM(ErrorData);

    /** apply error model */
    if(Config.RangeErrorModel.Type == ROBUST_STSM)
    {
      libRSF::SumMix1 NewSMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Range2, NewSMModel);
    }
    else if(Config.RangeErrorModel.Type == ROBUST_STMM)
    {
      libRSF::MaxMix1 NewMMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Range2, NewMMModel);
    }
  }
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  libRSF::FactorGraphConfig Config;

  /** process command line parameter */
  if(!Config.ReadCommandLineOptions(argc, argv))
  {
    std::cout << std::endl;
    return 1;
  }

  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.max_num_iterations = 1000;
  SolverOptions.num_threads = 8;
  SolverOptions.max_solver_time_in_seconds = 0.25;

  const int NumberOfComponents = 2;

  /** read input data */
  libRSF::SensorDataSet InputData;
  libRSF::ReadDataFromFile(Config.InputFile, InputData);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;
  libRSF::SensorData DeltaTime;

  double Timestamp, TimestampFirst, TimestampOld, TimestampLast;
  InputData.getFirstTimestamp("Range 2D", TimestampFirst);
  InputData.getLastTimestamp("Range 2D", TimestampLast);
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
  libRSF::SensorData Odom = InputData.getElement("Differential Odometry 2D", Timestamp);
  libRSF::GaussianDiagonal<3> NoiseOdom2Diff(Odom.getStdDev());

  /** iterate over timestamps */
  while(InputData.getNextTimestamp("Range 2D", Timestamp, Timestamp))
  {
    DeltaTime = GenerateDeltaTime(TimestampOld, Timestamp);

    /** add required states */
    Graph.addState(POSITION_STATE, libRSF::StateType::Pose2, Timestamp);
    Graph.addState(ORIENTATION_STATE, libRSF::StateType::Angle, Timestamp);

    /** add motion model or odometry */
    libRSF::SensorData DeltaTime = GenerateDeltaTime(TimestampOld, Timestamp);

    libRSF::StateList MotionList;
    MotionList.add(POSITION_STATE, TimestampOld);
    MotionList.add(POSITION_STATE, Timestamp);
    MotionList.add(ORIENTATION_STATE, TimestampOld);
    MotionList.add(ORIENTATION_STATE, Timestamp);
    Graph.addFactor(libRSF::FactorType::Odom2Diff, MotionList, InputData.getElement("Differential Odometry 2D", Timestamp), DeltaTime, NoiseOdom2Diff);

    /** add all range measurements of with current timestamp */
    AddRangeMeasurements2D(Graph, InputData, Config, Timestamp);

    /** tune self-tuning error model */
    TuneErrorModel(Graph, Config, NumberOfComponents, Timestamp);

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
