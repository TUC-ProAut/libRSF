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
* @file Example_IMU.cpp
* @author Tim Pfeifer
* @date 27.03.2019
* @brief A simple example application to test the IMU pre-integration factor.
* @copyright GNU Public License.
*
*/

#include "libRSF.h"

#define POSITION_STATE "Position"
#define ORIENTATION_STATE "Orientation"
#define IMU_STATE "IMUBias"

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

  /** read input data */
  libRSF::SensorDataSet InputData;
  libRSF::ReadDataFromFile(Config.InputFile, InputData);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;

  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;

  SolverOptions.max_num_iterations = 1000;
  SolverOptions.max_solver_time_in_seconds = 10;
  SolverOptions.num_threads = 1;//std::thread::hardware_concurrency();

  /** get relevant timestamps*/
  double Timestamp, TimestampOld, TimestampFirst = 0, TimestampLast;
  InputData.getTimeFirst(libRSF::DataType::Point3, TimestampFirst);
  InputData.getTimeLast(libRSF::DataType::IMU, TimestampLast);
  TimestampOld = TimestampFirst;

  /** add states */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point3, TimestampOld);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimestampOld);
  Graph.addState(IMU_STATE, libRSF::DataType::IMUBias, TimestampOld);

  /** freeze first values */
  Graph.setConstant(POSITION_STATE, TimestampOld);
  Graph.setConstant(ORIENTATION_STATE, TimestampOld);
  Graph.setConstant(IMU_STATE, TimestampOld);

  /** create pre-integration object */
//  libRSF::IMUPreintegration Preintegrator(TimestampFirst);

  /** calculate delta time */
  double T0 = 0.0, T1 = 0.0;
  InputData.getTimeFirst(libRSF::DataType::IMU, T0);
  InputData.getTimeNext(libRSF::DataType::IMU, T0, T1);
  double dT = T1 - T0;

  /** create IMU error model */
  double StdDevAcc = Config.IMU.Parameter(0) * sqrt(1.0/dT);
  double StdDevVel = Config.IMU.Parameter(4);
  double StdDevTR = Config.IMU.Parameter(1) * sqrt(1.0/dT);
  double StdDevBiasAcc = Config.IMU.Parameter(2) * sqrt(dT);
  double StdDevBiasTR = Config.IMU.Parameter(3) * sqrt(dT);

  libRSF::Vector15 NoiseVect;
  NoiseVect << StdDevAcc,StdDevAcc,StdDevAcc,
               StdDevVel,StdDevVel,StdDevVel,
               StdDevTR,StdDevTR,StdDevTR,
               StdDevBiasAcc,StdDevBiasAcc,StdDevBiasAcc,
               StdDevBiasTR,StdDevBiasTR,StdDevBiasTR;

  libRSF::GaussianDiagonal<15> IMUNoise;
  IMUNoise.setStdDevDiagonal(NoiseVect);

  Timestamp = T0;
  do
  {
    /** add new states */
    Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, Timestamp);
    Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, Timestamp);
    Graph.addStateWithCheck(IMU_STATE, libRSF::DataType::IMUBias, Timestamp);

    /** add absolute measurement if available */
    libRSF::Data PositionMeasurement;
    if(InputData.getElement(libRSF::DataType::Point3, TimestampOld, 0, PositionMeasurement))
    {
      libRSF::StateList PositionStates;
      PositionStates.add(POSITION_STATE,TimestampOld);

      const libRSF::Matrix33 CovMat = PositionMeasurement.getCovarianceMatrix();
      libRSF::GaussianFull<3> PosNoise;
      PosNoise.setCovarianceMatrix(CovMat);

      Graph.addFactor<libRSF::FactorType::Prior3>(PositionStates, PositionMeasurement, PosNoise);

      /** the first state is defined if there are absolute measurements */
//      Graph.setVariable(POSITION_STATE, TimestampOld);
//      Graph.setVariable(ORIENTATION_STATE, TimestampOld);
//      Graph.setVariable(IMU_STATE, TimestampOld);

      /** set position to measurement */
//      Graph.getStateData().getElement(POSITION_STATE, TimestampOld,0).setMean(PositionMeasurement.getMean());
    }
    /** add simple IMU factor */
    libRSF::StateList IMUStates;
    IMUStates.add(POSITION_STATE,TimestampOld);
    IMUStates.add(ORIENTATION_STATE,TimestampOld);
    IMUStates.add(IMU_STATE,TimestampOld);
    IMUStates.add(POSITION_STATE,Timestamp);
    IMUStates.add(ORIENTATION_STATE,Timestamp);
    IMUStates.add(IMU_STATE,Timestamp);

    libRSF::IMUPreintegrator PreInt(libRSF::Vector3::Zero(),libRSF::Vector3::Zero(),
                                    Config.IMU.Parameter(0), Config.IMU.Parameter(1),
                                    Config.IMU.Parameter(2) ,Config.IMU.Parameter(3),
                                    TimestampOld);

    PreInt.addMeasurement(InputData.getElement(libRSF::DataType::IMU, Timestamp));
    Graph.addIMUPreintegrationFactor(IMUStates, PreInt.getPreintegratedState());

//    Graph.addFactor<libRSF::FactorType::IMUSimple>(IMUStates, InputData.getElement(libRSF::DataType::IMU, Timestamp), IMUNoise);

    TimestampOld = Timestamp;
  }
  while(InputData.getTimeNext(libRSF::DataType::IMU, Timestamp, Timestamp));

  /** add last measurement */
  libRSF::Data PositionMeasurement;
  if(InputData.getElement(libRSF::DataType::Point3, TimestampOld, 0, PositionMeasurement))
  {
    libRSF::StateList PositionStates;
    PositionStates.add(POSITION_STATE,TimestampOld);

    const libRSF::Matrix33 CovMat = PositionMeasurement.getCovarianceMatrix();
    libRSF::GaussianFull<3> PosNoise;
    PosNoise.setCovarianceMatrix(CovMat);

    Graph.addFactor<libRSF::FactorType::Prior3>(PositionStates, PositionMeasurement, PosNoise);
  }

  SolverOptions.minimizer_progress_to_stdout = true;
  Graph.solve(SolverOptions);
  Graph.printReport();


  Graph.computeCovariance(POSITION_STATE);
  Result = Graph.getStateData();

  /** write results to disk */
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result);
  return 0;
}
