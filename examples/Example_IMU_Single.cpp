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
* @file Example_IMU_Single.cpp
* @author Tim Pfeifer
* @date 27.08.2020
* @brief An example application to test the IMU factors with a single measurement.
* @copyright GNU Public License.
*
*/

#include "libRSF.h"

#define POSITION_STATE "Position"
#define ORIENTATION_STATE "Orientation"
#define IMU_STATE "IMUBias"

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** create measurement mean */
  libRSF::Vector6 MeanIMU;
  MeanIMU << 0.1, 0, 0, 0, 0, 0.1;

  double dt = 0.5;

  /** add gravity */
  MeanIMU(2) += 9.81;

  /** create sensor object */
  libRSF::Data IMU(libRSF::DataType::IMU, dt);
  IMU.setMean(MeanIMU);

  /** create graph */
  libRSF::FactorGraph Graph;

  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, 0.0);
  Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, 0.0);
  Graph.addStateWithCheck(IMU_STATE, libRSF::DataType::IMUBias, 0.0);

  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, dt);
  Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, dt);
  Graph.addStateWithCheck(IMU_STATE, libRSF::DataType::IMUBias, dt);

  /** freeze first values */
  Graph.setConstant(POSITION_STATE, 0.0);
  Graph.setConstant(ORIENTATION_STATE, 0.0);
  Graph.setConstant(IMU_STATE, 0.0);

  /** pre-integrate measurements */
  libRSF::IMUPreintegrator PreInt(libRSF::Vector3::Zero(),/**< bias accelerometer  */
                                  libRSF::Vector3::Zero(),/**< bias gyrometer */
                                  1.0,  /**< noise density acc. [m/s^2/sqrt(Hz)]*/
                                  1.0,  /**< noise density gyro [rad/s/sqrt(Hz)]*/
                                  1.0,  /**< random walk acc. [m/s^2]*/
                                  1.0,  /**< random walk gyro. [rad/s]*/
                                  0.0); /**< current timestamp */
  IMU.setTimestamp(dt);
  PreInt.addMeasurement(IMU); /** there is only one IMU measurement in this example */

  /** add factor */
  libRSF::StateList IMUStates;
  IMUStates.add(POSITION_STATE, 0.0);
  IMUStates.add(ORIENTATION_STATE, 0.0);
  IMUStates.add(IMU_STATE, 0.0);
  IMUStates.add(POSITION_STATE, dt);
  IMUStates.add(ORIENTATION_STATE, dt);
  IMUStates.add(IMU_STATE, dt);
  Graph.addIMUPreintegrationFactor(IMUStates, PreInt.getPreintegratedState());

  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = true;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;

  SolverOptions.max_num_iterations = 1000;
  SolverOptions.max_solver_time_in_seconds = 10;
  SolverOptions.num_threads = 1;

  /** optimize */
  Graph.solve(SolverOptions);
  Graph.printReport();
  Graph.computeCovariance(POSITION_STATE, dt);
  Graph.computeCovariance(ORIENTATION_STATE, dt);
  Graph.computeCovariance(IMU_STATE, dt);

  /** print result */
  std::cout << "##### estimated states #####" << std::endl;
  std::cout << "Translation:" << std::endl
            << Graph.getStateData().getElement(POSITION_STATE, dt).getMean().transpose()
            << std::endl << "Covariance:"<< std::endl
            << libRSF::MatrixRef<double,3,3>(Graph.getStateData().getElement(POSITION_STATE, dt).getDataPointer(libRSF::DataElement::Covariance))
            << std::endl << std::endl;

  std::cout << "Rotation:" << std::endl
            << Graph.getStateData().getElement(ORIENTATION_STATE, dt).getMean().transpose()
            << std::endl << "Covariance:"<< std::endl
            << libRSF::MatrixRef<double,4,4>(Graph.getStateData().getElement(ORIENTATION_STATE, dt).getDataPointer(libRSF::DataElement::Covariance))
            << std::endl << std::endl;

  std::cout << "Speedbias:" << std::endl
            << Graph.getStateData().getElement(IMU_STATE, dt).getMean().transpose()
            << std::endl << "Covariance:"<< std::endl
            << libRSF::MatrixRef<double,9,9>(Graph.getStateData().getElement(IMU_STATE, dt).getDataPointer(libRSF::DataElement::Covariance));

  std::cout << std::endl << "##### pre-integrated  values #####" << std::endl;
  std::cout << "Translation:" << std::endl << PreInt.getPreintegratedState().Translation.transpose()  << std::endl << std::endl;
  std::cout << "Rotation:" << std::endl << PreInt.getPreintegratedState().Rotation.coeffs().transpose() << std::endl << std::endl;
  std::cout << "Velocity:" << std::endl << PreInt.getPreintegratedState().Velocity.transpose()  << std::endl;
  std::cout << std::endl << "Covariance:" << std::endl << PreInt.getPreintegratedState().PreIntCov << std::endl << std::endl;

  std::cout << std::endl << "Jacobians" << std::endl;
  std::cout << "J_TA: " << std::endl << PreInt.getPreintegratedState().JacTransBiasAcc << std::endl << std::endl;
  std::cout << "J_TG: " << std::endl << PreInt.getPreintegratedState().JacVelBiasTR << std::endl << std::endl;
  std::cout << "J_VA: " << std::endl << PreInt.getPreintegratedState().JacVelBiasAcc << std::endl << std::endl;
  std::cout << "J_VG: " << std::endl << PreInt.getPreintegratedState().JacVelBiasTR << std::endl << std::endl;
  std::cout << "J_RG: " << std::endl << PreInt.getPreintegratedState().JacRotBiasTRLocal << std::endl << std::endl;

  return 0;
}
