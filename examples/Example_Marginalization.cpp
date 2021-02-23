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
* @file Example_Marginalization.cpp
* @author Tim Pfeifer
* @date 29.11.2019
* @brief An example application to demonstrate that the Marginalization works.
* @copyright GNU Public License.
*
*/

#include <ceres/ceres.h>
#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position"
#define ROTATION_STATE "Quaternion"

#define STDDEV_POS    0.5
#define STDDEV_ROT    0.2

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(ArgV[0]);

  ceres::Solver::Options FGOptions;
  FGOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  FGOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;

  /** set uncertainty */
  libRSF::Vector3 NoisePosVec, NoiseRotVec;
  NoisePosVec.fill(STDDEV_POS);
  NoiseRotVec.fill(STDDEV_ROT);

  libRSF::Vector6 NoisePoseVec;
  NoisePoseVec << NoisePosVec , NoiseRotVec;

  /** create noise models */
  libRSF::GaussianDiagonal<3> GaussPos, GausRot;
  GaussPos.setStdDevSharedDiagonal(STDDEV_POS);
  GausRot.setStdDevSharedDiagonal(STDDEV_ROT);

  libRSF::GaussianDiagonal<6> GaussPose;
  GaussPose.setStdDevDiagonal(NoisePoseVec);

  /** create desired states */
  libRSF::Vector3 PosVec0, PosVec1;
  PosVec0 << 0, 0, 0;
  PosVec1 << 1, 0.5, 0.25;

  libRSF::Quaternion Quat0, Quat1;
  Quat0.setIdentity();
  Quat1 = Quat0 * (Eigen::AngleAxisd(1, libRSF::Vector3::UnitX())
                 * Eigen::AngleAxisd(0, libRSF::Vector3::UnitY())
                 * Eigen::AngleAxisd(0, libRSF::Vector3::UnitZ()));

  libRSF::Vector4 RotVec0, RotVec1;
  RotVec0 << Quat0.vec(), Quat0.w();
  RotVec1 << Quat1.vec(), Quat1.w();

  libRSF::Vector7 PoseVec0, PoseVec1;
  PoseVec0 << PosVec0, Quat0.vec(), Quat0.w();
  PoseVec1 << PosVec1, Quat1.vec(), Quat1.w();

  /** create state objects as GT */
  libRSF::StateData StatePos0 (libRSF::StateType::Point3, 0.0);
  StatePos0.setMean(PosVec0);
  libRSF::StateData StatePos1 (libRSF::StateType::Point3, 1.0);
  StatePos1.setMean(PosVec1);

  libRSF::StateData StateRot0 (libRSF::StateType::Quat, 0.0);
  StateRot0.setMean(RotVec0);
  libRSF::StateData StateRot1 (libRSF::StateType::Quat, 1.0);
  StateRot1.setMean(RotVec1);

  /** calculate relative transformations */
  libRSF::Vector3 PosVecRel = PosVec1 - PosVec0;
  libRSF::Quaternion QuatRel = Quat0.conjugate()*Quat1;

  libRSF::Vector7 PoseVecRel;
  PoseVecRel << Quat0.conjugate()*PosVecRel, QuatRel.vec(), QuatRel.w();

  /** create measurement objects */
  libRSF::SensorData Pos0(libRSF::SensorType::Point3, 0.0);
  libRSF::SensorData PosRel(libRSF::SensorType::Point3, 1.0);

  Pos0.setMean(PosVec0);
  PosRel.setMean(PosVecRel);

  libRSF::SensorData Rot0(libRSF::SensorType::Quaternion, 0.0);
  libRSF::SensorData RotRel(libRSF::SensorType::Quaternion, 1.0);

  Rot0.setMean(RotVec0);
  RotRel.setMean((libRSF::Vector4() << QuatRel.vec(), QuatRel.w()).finished());

  libRSF::SensorData Pose0(libRSF::SensorType::Pose3, 0.0);
  libRSF::SensorData PoseRel(libRSF::SensorType::Pose3, 1.0);

  Pose0.setMean(PoseVec0);
  PoseRel.setMean(PoseVecRel);

  /** construct graph */
  libRSF::FactorGraph PositionGraph;
  libRSF::FactorGraph RotationGraph;
  libRSF::FactorGraph PoseGraph;

  /** add states */
  PositionGraph.addState(POSITION_STATE, libRSF::StateType::Point3, 0.0);
  PositionGraph.addState(POSITION_STATE, libRSF::StateType::Point3, 1.0);

  RotationGraph.addState(ROTATION_STATE, libRSF::StateType::Quat, 0.0);
  RotationGraph.addState(ROTATION_STATE, libRSF::StateType::Quat, 1.0);

  /** add factors */
  libRSF::StateID IDPos0(POSITION_STATE, 0.0, 0);
  libRSF::StateID IDPos1(POSITION_STATE, 1.0, 0);

  libRSF::StateID IDRot0(ROTATION_STATE, 0.0, 0);
  libRSF::StateID IDRot1(ROTATION_STATE, 1.0, 0);

  PositionGraph.addFactor<libRSF::FactorType::Prior3>(IDPos0, Pos0, GaussPos);
  PositionGraph.addFactor<libRSF::FactorType::BetweenValue3>(IDPos0, IDPos1, PosRel, GaussPos);

  RotationGraph.addFactor<libRSF::FactorType::PriorQuat>(IDRot0, Rot0, GausRot);
  RotationGraph.addFactor<libRSF::FactorType::BetweenQuaternion>(IDRot0, IDRot1, RotRel, GausRot);

  /** optimize */
  PositionGraph.solve(FGOptions);
  PositionGraph.computeCovariance(POSITION_STATE);

  RotationGraph.solve(FGOptions);
  RotationGraph.computeCovariance(ROTATION_STATE);
//  RotationGraph.printReport();

  /** store results */
  libRSF::StateDataSet DataGT;
  libRSF::StateDataSet DataFGFull;
  libRSF::StateDataSet DataFGMarg;

  DataGT.addElement(POSITION_STATE,StatePos0);
  DataGT.addElement(POSITION_STATE,StatePos1);

  DataGT.addElement(ROTATION_STATE,StateRot0);
  DataGT.addElement(ROTATION_STATE,StateRot1);

  DataFGFull.addElement(POSITION_STATE, PositionGraph.getStateData().getElement(POSITION_STATE, 0.0, 0));
  DataFGFull.addElement(POSITION_STATE, PositionGraph.getStateData().getElement(POSITION_STATE, 1.0, 0));

  DataFGFull.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 0.0, 0));
  DataFGFull.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 1.0, 0));

  /** marginalize */
  PositionGraph.marginalizeState(POSITION_STATE, 0.0, 0);
  RotationGraph.marginalizeState(ROTATION_STATE, 0.0, 0);

  /** check covariance again */
  PositionGraph.solve(FGOptions);
  PositionGraph.computeCovariance(POSITION_STATE);
  DataFGMarg.addElement(POSITION_STATE, PositionGraph.getStateData().getElement(POSITION_STATE, 1.0, 0));

  RotationGraph.solve(FGOptions);
  RotationGraph.computeCovariance(ROTATION_STATE);
  DataFGMarg.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 1.0, 0));

  /** print results position */
  std::cout << "############################################" << std::endl;
  std::cout << "############## Position Graph ##############" << std::endl;
  std::cout << "############################################" << std::endl << std::endl;

  std::cout << "################### GT #####################" << std::endl;
  std::cout << DataGT.getElement(POSITION_STATE, 0.0, 0).getNameValueString() << std::endl;
  std::cout << DataGT.getElement(POSITION_STATE, 1.0, 0).getNameValueString() << std::endl << std::endl;

  std::cout << "################ FGO full ##################" << std::endl;
  std::cout << DataFGFull.getElement(POSITION_STATE, 0.0, 0).getNameValueString() << std::endl;
  std::cout << DataFGFull.getElement(POSITION_STATE, 1.0, 0).getNameValueString() << std::endl << std::endl;

  std::cout << "################ FGO marg ##################" << std::endl;
  std::cout << DataFGMarg.getElement(POSITION_STATE, 1.0, 0).getNameValueString() << std::endl << std::endl;

  /** print results rotation */
  std::cout << "############################################" << std::endl;
  std::cout << "############## Rotation Graph ##############" << std::endl;
  std::cout << "############################################" << std::endl << std::endl;

  std::cout << "################### GT #####################" << std::endl;
  std::cout << DataGT.getElement(ROTATION_STATE, 0.0, 0).getNameValueString() << std::endl;
  std::cout << DataGT.getElement(ROTATION_STATE, 1.0, 0).getNameValueString() << std::endl << std::endl;

  std::cout << "################ FGO full ##################" << std::endl;
  std::cout << DataFGFull.getElement(ROTATION_STATE, 0.0, 0).getNameValueString() << std::endl;
  std::cout << DataFGFull.getElement(ROTATION_STATE, 1.0, 0).getNameValueString() << std::endl << std::endl;

  std::cout << "################ FGO marg ##################" << std::endl;
  std::cout << DataFGMarg.getElement(ROTATION_STATE, 1.0, 0).getNameValueString() << std::endl << std::endl;


  return 0;
}
