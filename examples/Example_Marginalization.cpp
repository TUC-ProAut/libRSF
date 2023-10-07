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
* @file Example_Marginalization.cpp
* @author Tim Pfeifer
* @date 29.11.2019
* @brief An example application to demonstrate that the Marginalization works.
* @copyright GNU Public License.
*
*/

#include "Example_Marginalization.h"

void CreateData(libRSF::StateDataSet &GT, libRSF::SensorDataSet &Measurements)
{
  /** create ground truth */
  libRSF::Vector3 TransVec0, TransVec1;
  TransVec0 << 0, 0, 0;
  TransVec1 << 1, -0.5, 0.25;

  libRSF::Quaternion Quat0, Quat1;
  Quat0.setIdentity();
  Quat1 = Quat0 * (Eigen::AngleAxisd(30, libRSF::Vector3::UnitX())
                 * Eigen::AngleAxisd(-60, libRSF::Vector3::UnitY())
                 * Eigen::AngleAxisd(100, libRSF::Vector3::UnitZ()));

  libRSF::Vector4 RotVec0, RotVec1;
  RotVec0 << Quat0.vec(), Quat0.w();
  RotVec1 << Quat1.vec(), Quat1.w();

  /** convert GT to state objects*/
  libRSF::Data TransState0 (libRSF::DataType::Point3, 0.0);
  TransState0.setMean(TransVec0);
  libRSF::Data TransState1 (libRSF::DataType::Point3, 1.0);
  TransState1.setMean(TransVec1);

  libRSF::Data RotState0 (libRSF::DataType::Quaternion, 0.0);
  RotState0.setMean(RotVec0);
  libRSF::Data RotState1 (libRSF::DataType::Quaternion, 1.0);
  RotState1.setMean(RotVec1);

  /** add to ground truth data set */
  GT.addElement(POSITION_STATE,TransState0);
  GT.addElement(POSITION_STATE,TransState1);
  GT.addElement(ROTATION_STATE,RotState0);
  GT.addElement(ROTATION_STATE,RotState1);

  /** create small error for relative translation */
  const libRSF::Vector3 ErrorTrans = libRSF::Vector3::Ones()*0.1;
  const libRSF::Quaternion ErrorQuat = Eigen::AngleAxisd(10, libRSF::Vector3::UnitX())
                                     * Eigen::AngleAxisd(10, libRSF::Vector3::UnitY())
                                     * Eigen::AngleAxisd(10, libRSF::Vector3::UnitZ());

  /** calculate relative transformations */
  const libRSF::Vector3 TransRelVec = TransVec1 - TransVec0 + ErrorTrans;
  const libRSF::Quaternion QuatRel = Quat0.conjugate()*Quat1 * ErrorQuat;
  libRSF::Vector4 RotRelVec;
  RotRelVec << QuatRel.vec(), QuatRel.w();

  /** create measurement objects */
  libRSF::Data Trans0(libRSF::DataType::Point3, 0.0);
  libRSF::Data Trans1(libRSF::DataType::Point3, 1.0);
  libRSF::Data TransRel(libRSF::DataType::Point3, 0.5);

  Trans0.setMean(TransVec0);
  Trans1.setMean(TransVec1);
  TransRel.setMean(TransRelVec);

  libRSF::Data Rot0(libRSF::DataType::Quaternion, 0.0);
  libRSF::Data Rot1(libRSF::DataType::Quaternion, 1.0);
  libRSF::Data RotRel(libRSF::DataType::Quaternion, 0.5);

  Rot0.setMean(RotVec0);
  Rot1.setMean(RotVec1);
  RotRel.setMean(RotRelVec);

  /**< add measurement objects to dataset */
  Measurements.addElement(Trans0);
  Measurements.addElement(Trans1);
  Measurements.addElement(TransRel);

  Measurements.addElement(Rot0);
  Measurements.addElement(Rot1);
  Measurements.addElement(RotRel);
}

void CreateGraphs(libRSF::FactorGraph &TranslationGraph,
                  libRSF::FactorGraph &RotationGraph,
                  libRSF::FactorGraph &PoseGraph,
                  libRSF::SensorDataSet &Measurements)
{
  /** set uncertainty */
  libRSF::Vector3 NoisePosVec, NoiseRotVec;
  NoisePosVec.fill(STDDEV_POS);
  NoiseRotVec.fill(STDDEV_ROT);

  /** create noise models */
  libRSF::GaussianDiagonal<3> GaussPos, GaussRot;
  GaussPos.setStdDevSharedDiagonal(STDDEV_POS);
  GaussRot.setStdDevSharedDiagonal(STDDEV_ROT);

  /** add states */
  TranslationGraph.addState(POSITION_STATE, libRSF::DataType::Point3, 0.0);
  TranslationGraph.addState(POSITION_STATE, libRSF::DataType::Point3, 1.0);

  RotationGraph.addState(ROTATION_STATE, libRSF::DataType::Quaternion, 0.0);
  RotationGraph.addState(ROTATION_STATE, libRSF::DataType::Quaternion, 1.0);

  /** add factors */
  libRSF::StateID IDPos0(POSITION_STATE, 0.0, 0);
  libRSF::StateID IDPos1(POSITION_STATE, 1.0, 0);

  libRSF::StateID IDRot0(ROTATION_STATE, 0.0, 0);
  libRSF::StateID IDRot1(ROTATION_STATE, 1.0, 0);

  TranslationGraph.addFactor<libRSF::FactorType::Prior3>(IDPos0, Measurements.getElement(libRSF::DataType::Point3, 0.0), GaussPos);
  TranslationGraph.addFactor<libRSF::FactorType::Prior3>(IDPos1, Measurements.getElement(libRSF::DataType::Point3, 1.0), GaussPos);
  TranslationGraph.addFactor<libRSF::FactorType::BetweenValue3>(IDPos0, IDPos1, Measurements.getElement(libRSF::DataType::Point3, 0.5), GaussPos);

  RotationGraph.addFactor<libRSF::FactorType::PriorQuat>(IDRot0, Measurements.getElement(libRSF::DataType::Quaternion, 0.0), GaussRot);
  RotationGraph.addFactor<libRSF::FactorType::PriorQuat>(IDRot1, Measurements.getElement(libRSF::DataType::Quaternion, 1.0), GaussRot);
  RotationGraph.addFactor<libRSF::FactorType::BetweenQuaternion>(IDRot0, IDRot1, Measurements.getElement(libRSF::DataType::Quaternion, 0.5), GaussRot);
}

#ifndef TESTMODE // only compile main if not used in test context

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** create data */
  libRSF::StateDataSet DataGT;
  libRSF::SensorDataSet Measurements;
  CreateData(DataGT, Measurements);

  /** construct graph */
  libRSF::FactorGraph TranslationGraph;
  libRSF::FactorGraph RotationGraph;
  libRSF::FactorGraph PoseGraph;
  CreateGraphs(TranslationGraph, RotationGraph, PoseGraph, Measurements);

  /** configure solver */
  ceres::Solver::Options FGOptions;
  FGOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  FGOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  FGOptions.linear_solver_type = ceres::LinearSolverType::DENSE_QR;

  /** optimize */
  TranslationGraph.solve(FGOptions);
  TranslationGraph.computeCovariance(POSITION_STATE);

  RotationGraph.solve(FGOptions);
  RotationGraph.computeCovariance(ROTATION_STATE);

  /** store results */
  libRSF::StateDataSet DataFGFull;
  libRSF::StateDataSet DataFGMarg;

  DataFGFull.addElement(POSITION_STATE, TranslationGraph.getStateData().getElement(POSITION_STATE, 0.0, 0));
  DataFGFull.addElement(POSITION_STATE, TranslationGraph.getStateData().getElement(POSITION_STATE, 1.0, 0));

  DataFGFull.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 0.0, 0));
  DataFGFull.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 1.0, 0));

  /** marginalize */
  TranslationGraph.marginalizeState(POSITION_STATE, 0.0, 0);
  RotationGraph.marginalizeState(ROTATION_STATE, 0.0, 0);

  /** check covariance again */
  TranslationGraph.solve(FGOptions);
  TranslationGraph.computeCovariance(POSITION_STATE);
  DataFGMarg.addElement(POSITION_STATE, TranslationGraph.getStateData().getElement(POSITION_STATE, 1.0, 0));

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

#endif // TESTMODE
