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
 * @file Test_Example_.cpp
 * @author Leopold Mauersberger
 * @date 02 Mar 2021
 * @brief Comparing the  example results against sample solution
 * @copyright GNU Public License.
 *
 */

#include "../examples/Example_Marginalization.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

TEST(Example, Marginalization)
{
  /** create data */
  libRSF::StateDataSet DataGT;
  libRSF::SensorDataSet Measurements;
  CreateData(DataGT, Measurements);

  /** construct graph */
  libRSF::FactorGraph TranslationGraph;
  libRSF::FactorGraph RotationGraph;
  libRSF::FactorGraph PoseGraph; /**< TODO */
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

  /** get results before marginalization */
  libRSF::StateDataSet DataFGFull;

  DataFGFull.addElement(POSITION_STATE, TranslationGraph.getStateData().getElement(POSITION_STATE, 0.0, 0));
  DataFGFull.addElement(POSITION_STATE, TranslationGraph.getStateData().getElement(POSITION_STATE, 1.0, 0));

  DataFGFull.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 0.0, 0));
  DataFGFull.addElement(ROTATION_STATE, RotationGraph.getStateData().getElement(ROTATION_STATE, 1.0, 0));

  /** marginalize */
  TranslationGraph.marginalizeState(POSITION_STATE, 0.0, 0);
  RotationGraph.marginalizeState(ROTATION_STATE, 0.0, 0);

  /** get results again after marginalization */
  libRSF::StateDataSet DataFGMarg;

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

  /** create GT "sensor data" */
  libRSF::SensorDataSet ResultGT;
  ResultGT.addElement(DataGT.getElement(POSITION_STATE, 0.0));
  ResultGT.addElement(DataGT.getElement(POSITION_STATE, 1.0));
  ResultGT.addElement(DataGT.getElement(ROTATION_STATE, 0.0));
  ResultGT.addElement(DataGT.getElement(ROTATION_STATE, 1.0));

  /** create "sensor data" from the full graph */
  libRSF::SensorDataSet ResultFull;
  ResultFull.addElement(DataFGFull.getElement(POSITION_STATE, 1.0));
  ResultFull.addElement(DataFGFull.getElement(ROTATION_STATE, 1.0));


  /** evaluate if full graph converged */
  const double maxAbsErrorFullPos = libRSF::MaxAbsError(libRSF::DataType::Point3,
                                                        ResultGT,
                                                        POSITION_STATE,
                                                        DataFGFull,
                                                        libRSF::DataElement::Mean);
  const double maxAbsErrorFullRot = libRSF::MaxAbsError(libRSF::DataType::Quaternion,
                                                        ResultGT,
                                                        ROTATION_STATE,
                                                        DataFGFull,
                                                        libRSF::DataElement::Mean);

  /** evaluate if marginalization was successful */
  const double maxAbsErrorMeanPos = libRSF::MaxAbsError(libRSF::DataType::Point3,
                                                        ResultFull,
                                                        POSITION_STATE,
                                                        DataFGMarg,
                                                        libRSF::DataElement::Mean);

  const double maxAbsErrorCovPos = libRSF::MaxAbsError(libRSF::DataType::Point3,
                                                        ResultFull,
                                                        POSITION_STATE,
                                                        DataFGMarg,
                                                        libRSF::DataElement::Covariance);

  const double maxAbsErrorMeanRot = libRSF::MaxAbsError(libRSF::DataType::Quaternion,
                                                        ResultFull,
                                                        ROTATION_STATE,
                                                        DataFGMarg,
                                                        libRSF::DataElement::Mean);

  const double maxAbsErrorCovRot = libRSF::MaxAbsError(libRSF::DataType::Quaternion,
                                                        ResultFull,
                                                        ROTATION_STATE,
                                                        DataFGMarg,
                                                        libRSF::DataElement::Covariance);

  std::cout << "MaxAbsError Full<->GT of translation mean:             " << maxAbsErrorFullPos << std::endl;
  std::cout << "MaxAbsError Full<->GT of rotation mean:                " << maxAbsErrorFullRot << std::endl;
  std::cout << "MaxAbsError Full<->Marginal of translation mean:       " << maxAbsErrorMeanPos << std::endl;
  std::cout << "MaxAbsError Full<->Marginal of translation covariance: " << maxAbsErrorCovPos << std::endl;
  std::cout << "MaxAbsError Full<->Marginal of rotation mean:          " << maxAbsErrorMeanRot << std::endl;
  std::cout << "MaxAbsError Full<->Marginal of rotation covariance:    " << maxAbsErrorCovRot << std::endl;

  EXPECT_LT(maxAbsErrorFullPos,0.1);
  EXPECT_LT(maxAbsErrorFullRot,0.2);
  EXPECT_LT(maxAbsErrorCovPos,1e-3);
  EXPECT_LT(maxAbsErrorMeanPos,1e-3);
  EXPECT_LT(maxAbsErrorCovRot,1e-3);
  EXPECT_LT(maxAbsErrorMeanRot,1e-3);
}

/** main provided by linking to gtest_main */
