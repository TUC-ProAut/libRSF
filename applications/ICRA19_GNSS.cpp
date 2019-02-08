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
 * @file ICRA19_GNSS.cpp
 * @author Tim Pfeifer
 * @date 17 Sep 2018
 * @brief File containing an application for 3D pose estimation based on pseudo range measurements. This special version is made for ICRA 2019.
 * @copyright GNU Public License.
 *
 */

#include "ICRA19_GNSS.h"

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
 * @param Measurements reference to the dataset that contains the pseudo range measurements
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
  libRSF::FactorGraphConfig SimpleConfig = Config;
  SimpleConfig.RangeErrorModel.Type = ROBUST_NONE;
  libRSF::FactorGraph SimpleGraph;

  SimpleGraph.addState(POSITION_STATE, libRSF::StateType::Pose3, TimestampFirst);
  SimpleGraph.addState(CLOCK_ERROR_STATE, libRSF::StateType::Val1, TimestampFirst);
  AddPseudorangeMeasurements(SimpleGraph, Measurements, SimpleConfig, TimestampFirst);

  /** solve */
  SimpleGraph.solve(Options);

  /**add first state variables */
  Graph.addState(POSITION_STATE, libRSF::StateType::Pose3, TimestampFirst);
  Graph.addState(CLOCK_ERROR_STATE, libRSF::StateType::Val1, TimestampFirst);
  Graph.addState(CLOCK_DRIFT_STATE, libRSF::StateType::Val1, TimestampFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::StateType::Angle, TimestampFirst);

  /** copy values to real graph */
  Graph.getStateData().getElement(POSITION_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(POSITION_STATE, TimestampFirst).getMean());
  Graph.getStateData().getElement(CLOCK_ERROR_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(CLOCK_ERROR_STATE, TimestampFirst).getMean());


  /** add first set of measurements */
  AddPseudorangeMeasurements(Graph, Measurements, Config, TimestampFirst);
}


/** @brief Adds a pseudorange measurement to the graph
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the measurement
 * @param Config reference to the factor graph config object that specifies the motion model
 * @param Timestamp a double timestamp of the current position
 * @return nothing
 *
 */
void AddPseudorangeMeasurements(libRSF::FactorGraph &Graph,
                                libRSF::SensorDataSet &Measurements,
                                libRSF::FactorGraphConfig const &Config,
                                double Timestamp)
{
  libRSF::StateList ListPseudorange;
  libRSF::SensorData Pseudorange;
  libRSF::GaussianDiagonal<1> NoisePseudorange;

  ListPseudorange.add(POSITION_STATE, Timestamp);
  ListPseudorange.add(CLOCK_ERROR_STATE, Timestamp);

  size_t SatNumber = Measurements.countElement("Range 3D", Timestamp);

  for(size_t SatCounter = 0; SatCounter < SatNumber; ++SatCounter)
  {
    /** get measurement */
    Pseudorange = Measurements.getElement("Range 3D", Timestamp, SatCounter);

    /** default error model */
    static libRSF::GaussianMixture<1> GMM((ceres::Vector(2) << 0, 0).finished(),
                                           (ceres::Vector(2) << 10, 100).finished(),
                                           (ceres::Vector(2) << 0.5, 0.5).finished());

    /** add factor */
    switch(Config.RangeErrorModel.Type)
    {
    case ROBUST_NONE:
      NoisePseudorange.setStdDev(Pseudorange.getStdDev());
      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorange);
      break;

    case ROBUST_DCS:
      NoisePseudorange.setStdDev(Pseudorange.getStdDev());
      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorange, new libRSF::DCSLoss(1.0));
      break;

    case ROBUST_CDCE:
      NoisePseudorange.setStdDev(Eigen::Matrix<double, 1, 1>::Ones());
      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorange, new libRSF::cDCELoss(Pseudorange.getStdDev()[0]));
      break;

    case ROBUST_MM:
      static libRSF::MaxMix1 NoisePseudorangeMaxMix(GMM);
      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeMaxMix);

      break;

    case ROBUST_SM:
      static libRSF::SumMix1 NoisePseudorangeSumMix(GMM);

      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeSumMix);

      break;

    case ROBUST_STSM:
      static libRSF::SumMix1 NoisePseudorangeSumMixST(GMM);
      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeSumMixST);

      break;

    case ROBUST_STMM:
      static libRSF::MaxMix1 NoisePseudorangeMaxMixST(GMM);
      Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeMaxMixST);

      break;

    default:
      std::cerr << "Error: Wrong pseudorange error model model: " << Config.RangeErrorModel.Type << std::endl;
      break;
    }
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

        Component.setParamsStdDev((ceres::Vector(1) << 10).finished()*pow(10, nComponent),
                                  (ceres::Vector(1) << 0.0).finished(),
                                  (ceres::Vector(1) << 1.0/NumberOfComponents).finished());

        GMM.addComponent(Component);
      }
    }

    Graph.computeUnweightedError(libRSF::FactorType::Pseudorange3, ErrorData);

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

    /** remove offset of the first "LOS" component */
    GMM.removeOffset();

    /** apply error model */
    if(Config.RangeErrorModel.Type == ROBUST_STSM)
    {
      libRSF::SumMix1 NewSMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3, NewSMModel);
    }
    else if(Config.RangeErrorModel.Type == ROBUST_STMM)
    {
      libRSF::MaxMix1 NewMMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3, NewMMModel);
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
  InputData.getFirstTimestamp("Range 3D", TimestampFirst);
  InputData.getLastTimestamp("Range 3D", TimestampLast);
  Timestamp = TimestampFirst;
  TimestampOld = TimestampFirst;
  int nTimestamp = 0;

  /** add fist variables and factors */
  InitGraph(Graph, InputData, Config, SolverOptions, TimestampFirst);

  /** solve multiple times with refined model to achieve good initial convergence */
  Graph.solve(SolverOptions);
  TuneErrorModel(Graph, Config, NumberOfComponents, Timestamp);
  Graph.solve(SolverOptions);

  /** safe result at first timestamp */
  Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Timestamp, 0));

  /** get odometry noise from first measurement */
  libRSF::SensorData Odom = InputData.getElement("Odometry 3D", Timestamp);
  ceres::Vector StdOdom4DOF(4);
  StdOdom4DOF << Odom.getStdDev().head(3), Odom.getStdDev().tail(1);
  libRSF::GaussianDiagonal<4> NoiseOdom4DOF(StdOdom4DOF);

  /** hard coded constant clock error drift (CCED) model noise properties */
  ceres::Vector StdCCED(2);
  if(strcmp(Config.InputFile, "Data_Chemnitz.txt") == 0)
  {
    StdCCED << 0.1, 0.009; /** set CCED standard deviation for Chemnitz City dataset */
  }
  else
  {
    StdCCED << 0.05, 0.01; /** set CCED standard deviation for smartLoc datasets */
  }
  libRSF::GaussianDiagonal<2> NoiseCCED(StdCCED);

  /** iterate over timestamps */
  while(InputData.getNextTimestamp("Range 3D", Timestamp, Timestamp))
  {
    DeltaTime = GenerateDeltaTime(TimestampOld, Timestamp);

    /** add position, orientation and clock error */
    Graph.addState(POSITION_STATE, libRSF::StateType::Pose3, Timestamp);
    Graph.addState(ORIENTATION_STATE, libRSF::StateType::Angle, Timestamp);
    Graph.addState(CLOCK_ERROR_STATE, libRSF::StateType::Val1, Timestamp);
    Graph.addState(CLOCK_DRIFT_STATE, libRSF::StateType::Val1, Timestamp);

    /** add odometry */
    libRSF::StateList MotionList;
    MotionList.add(POSITION_STATE, TimestampOld);
    MotionList.add(POSITION_STATE, Timestamp);
    MotionList.add(ORIENTATION_STATE, TimestampOld);
    MotionList.add(ORIENTATION_STATE, Timestamp);
    Graph.addFactor(libRSF::FactorType::Odom4, MotionList, InputData.getElement("Odometry 3D", Timestamp), DeltaTime, NoiseOdom4DOF);

    /** add clock drift model */
    libRSF::StateList ClockList;
    ClockList.add(CLOCK_ERROR_STATE, TimestampOld);
    ClockList.add(CLOCK_ERROR_STATE, Timestamp);
    ClockList.add(CLOCK_DRIFT_STATE, TimestampOld);
    ClockList.add(CLOCK_DRIFT_STATE, Timestamp);
    Graph.addFactor(libRSF::FactorType::ConstDrift1, ClockList, DeltaTime, NoiseCCED);

    /** add all pseudo range measurements of with current timestamp */
    AddPseudorangeMeasurements(Graph, InputData, Config, Timestamp);

    /** tune self-tuning error model */
    TuneErrorModel(Graph, Config, NumberOfComponents, Timestamp);

    /** solve the estimation problem */
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
