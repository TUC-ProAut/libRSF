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
 * @file IV19_GNSS.cpp
 * @author Tim Pfeifer
 * @date 04 Jun 2019
 * @brief File containing an application for 3D pose estimation based on pseudo range measurements. This special version is made for the Intelligent Vehicles Symposium 2019.
 * @copyright GNU Public License.
 *
 */

#include "IV19_GNSS.h"

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
  Options.minimizer_progress_to_stdout = false;
  SimpleGraph.solve(Options);

  /**add first state variables */
  Graph.addState(POSITION_STATE, libRSF::StateType::Pose3, TimestampFirst);
  Graph.addState(CLOCK_ERROR_STATE, libRSF::StateType::Val1, TimestampFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::StateType::Angle, TimestampFirst);
  Graph.addState(CLOCK_DRIFT_STATE, libRSF::StateType::Val1, TimestampFirst);

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
        static libRSF::GaussianMixture<1> GMM_MM;
        if (GMM_MM.getNumberOfComponents() == 0)
        {
          InitGMM(GMM_MM, GMM_N, 10);
        }
        static libRSF::MaxMix1 NoisePseudorangeMaxMix(GMM_MM);

        Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeMaxMix);

        break;

      case ROBUST_SM:
        static libRSF::GaussianMixture<1> GMM_SM;
        if (GMM_SM.getNumberOfComponents() == 0)
        {
          InitGMM(GMM_SM, GMM_N, 10);
        }
        static libRSF::SumMix1 NoisePseudorangeSumMix(GMM_SM);

        Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeSumMix);

        break;

      case ROBUST_STSM:
        static libRSF::GaussianMixture<1> GMM_SMST;
        if (GMM_SMST.getNumberOfComponents() == 0)
        {
          InitGMM(GMM_SMST, GMM_N, 10);
        }
        static libRSF::SumMix1 NoisePseudorangeSumMixST(GMM_SMST);

        Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeSumMixST);

        break;

      case ROBUST_STMM:
        static libRSF::GaussianMixture<1> GMM_MMST;
        if (GMM_MMST.getNumberOfComponents() == 0)
        {
          InitGMM(GMM_MMST, GMM_N, 10);
        }
        static libRSF::MaxMix1 NoisePseudorangeMaxMixST(GMM_MMST);

        Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeMaxMixST);

        break;

      case ROBUST_STSM_VBI:
          /** fill empty GMM */
          static libRSF::GaussianMixture<1> GMM_SMST_VBI;
          if (GMM_SMST_VBI.getNumberOfComponents() == 0)
          {
            InitGMM(GMM_SMST_VBI, 2, 10);
          }
          static libRSF::SumMix1 NoisePseudorangeSumMixST_VBI(GMM_SMST_VBI);

          Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeSumMixST_VBI);

          break;

      case ROBUST_STMM_VBI:
          /** fill empty GMM */
          static libRSF::GaussianMixture<1> GMM_MMST_VBI;
          if (GMM_MMST_VBI.getNumberOfComponents() == 0)
          {
            InitGMM(GMM_MMST_VBI, 2, 10);
          }
          static libRSF::MaxMix1 NoisePseudorangeMaxMixST_VBI(GMM_MMST_VBI);

          Graph.addFactor(libRSF::FactorType::Pseudorange3, ListPseudorange, Pseudorange, NoisePseudorangeMaxMixST_VBI);

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
* @param Timestamp current timestamp
* @return nothing
*
*/
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config,
                    double Timestamp)
{
  if(Config.RangeErrorModel.Type == ROBUST_STSM || Config.RangeErrorModel.Type == ROBUST_STMM ||
      Config.RangeErrorModel.Type == ROBUST_STSM_VBI || Config.RangeErrorModel.Type == ROBUST_STMM_VBI)
  {
    /** compute resudiuals of the factor graph */
    std::vector<double> ErrorData;
    Graph.computeUnweightedError(libRSF::FactorType::Pseudorange3, ErrorData);

    /** remove zeros in MM results (unused dimension!) */
    if(Config.RangeErrorModel.Type == ROBUST_STMM || Config.RangeErrorModel.Type == ROBUST_STMM_VBI)
    {
      for(int i = ErrorData.size() - 1; i > 0; i -= 2)
      {
        ErrorData.erase(ErrorData.begin() + i);
      }
    }

    if(Config.RangeErrorModel.Type == ROBUST_STSM || Config.RangeErrorModel.Type == ROBUST_STMM)
    {
      /** fill empty GMM */
      libRSF::GaussianMixture<1> GMM;

      if(GMM.getNumberOfComponents() == 0)
      {
        InitGMM(GMM, GMM_N, 10);
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
    else if(Config.RangeErrorModel.Type == ROBUST_STSM_VBI || Config.RangeErrorModel.Type == ROBUST_STMM_VBI)
    {
      /** initialize GMM */
      static libRSF::GaussianMixture<1> GMMAdaptive;
      libRSF::GaussianComponent<1> Component;

      if(GMMAdaptive.getNumberOfComponents() == 0)
      {
        Component.setParamsStdDev((ceres::Vector(1) << 10.0).finished(),
                                  (ceres::Vector(1) << 0.0).finished(),
                                  (ceres::Vector(1) << 1.0).finished());
        GMMAdaptive.addComponent(Component);
      }

      /** calculate statistics for GMM initialization*/
      std::vector<double> v = ErrorData;
      double sum = std::accumulate(v.begin(), v.end(), 0.0);
      double mean = sum / v.size();

      std::vector<double> diff(v.size());
      std::transform(v.begin(), v.end(), diff.begin(), [mean](double x)
      {
        return x - mean;
      });
      double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      double stdev = std::sqrt(sq_sum / v.size());

      /** add just one component per timestamp */
      if(GMMAdaptive.getNumberOfComponents() >= VBI_N_MAX)
      {
        GMMAdaptive.sortComponentsByWeight();
        GMMAdaptive.removeLastComponent();
      }

      Component.setParamsStdDev((ceres::Vector(1) << stdev).finished(),
                                (ceres::Vector(1) << 0.0).finished(),
                                (ceres::Vector(1) << 1.0/GMMAdaptive.getNumberOfComponents()).finished());


      GMMAdaptive.addComponent(Component);

      /** estimate GMM with Variational Inference */
      GMMAdaptive.estimateWithVBI(ErrorData, VBI_NU);

      /** remove offset*/
      GMMAdaptive.removeOffset();

      /** apply error model */
      if(Config.RangeErrorModel.Type == ROBUST_STSM_VBI)
      {
        libRSF::SumMix1 NewSMModel(GMMAdaptive);
        Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3, NewSMModel);
      }
      else if(Config.RangeErrorModel.Type == ROBUST_STMM_VBI)
      {
        libRSF::MaxMix1 NewMMModel(GMMAdaptive);
        Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3, NewMMModel);
      }
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
  SolverOptions.num_threads = 8;
  SolverOptions.max_num_iterations = 100;

  /** read input data */
  libRSF::SensorDataSet InputData;
  libRSF::ReadDataFromFile(Config.InputFile, InputData);

  /** Build optimization problem from sensor data */
  ceres::Problem::Options Options;
  Options.enable_fast_removal = true;
  libRSF::FactorGraph Graph(Options);

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
  TuneErrorModel(Graph, Config, Timestamp);
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
    Graph.addState(CLOCK_ERROR_STATE, libRSF::StateType::Val1, Timestamp);
    Graph.addState(ORIENTATION_STATE, libRSF::StateType::Angle, Timestamp);
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
    TuneErrorModel(Graph, Config, Timestamp);

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
