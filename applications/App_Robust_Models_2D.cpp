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
* @file App_Robust_Models_2D.cpp
* @author Tim Pfeifer
* @date 27.08.2019
* @brief An simple application to evaluate the robust error functions in 2D.
* @copyright GNU Public License.
*
*/

#include "App_Robust_Models_2D.h"

int CreateGraphAndSolve(std::vector<std::string> &Arguments,
                       libRSF::StateDataSet &CostSurfaceData,
                       libRSF::StateDataSet &PreOptimizationData,
                       libRSF::StateDataSet &PostOptimizationData,
                       libRSF::StateDataSet &SolverData)
{
  /** parse testing parameter */
  const int NumberPoints = std::stoi(Arguments.at(3));
  const double Range = std::stod(Arguments.at(4));
  const string ErrorModel = Arguments.at(5);

  /** parse GMM parameter */
  libRSF::Vector2 Mean1, Mean2;
  Mean1 << std::stod(Arguments.at(6)), std::stod(Arguments.at(7));
  Mean2 << std::stod(Arguments.at(8)), std::stod(Arguments.at(9));

  libRSF::Matrix22 StdDev1, StdDev2;
  StdDev1 << std::stod(Arguments.at(10)), std::stod(Arguments.at(11)),
             std::stod(Arguments.at(12)), std::stod(Arguments.at(13));
  StdDev2 << std::stod(Arguments.at(14)), std::stod(Arguments.at(15)),
             std::stod(Arguments.at(16)), std::stod(Arguments.at(17));

  libRSF::Vector1 Weight1, Weight2;
  Weight1 << std::stod(Arguments.at(18));
  Weight2 << std::stod(Arguments.at(19));

  /** estimate DCS parameter */
  const double ScalingDCS = std::pow(10, Mean2(0));

  /** create our own graph object */
  libRSF::FactorGraph SimpleGraph;

  /** set the solver options for ceres */
  ceres::Solver::Options SolverOptions;
  SolverOptions.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
  SolverOptions.minimizer_type = ceres::MinimizerType::TRUST_REGION;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
  SolverOptions.num_threads = 1;
  SolverOptions.max_num_iterations = 100;
  SolverOptions.max_solver_time_in_seconds = 1.0;
  SolverOptions.minimizer_progress_to_stdout = false;

  /** decrease tolerances for an accurate result */
  SolverOptions.function_tolerance = 1e-8;
  SolverOptions.gradient_tolerance = SolverOptions.function_tolerance * 1e-4;

  /** configure Gaussian error model */
  libRSF::GaussianFull<2> Noise;
  Noise.setCovarianceMatrix(StdDev1.transpose()*StdDev1);

  /** configure Gaussian Identity model for cDCE */
  libRSF::GaussianDiagonal<2> NoiseIdentity;
  NoiseIdentity.setStdDevSharedDiagonal(1.0);

  /** create Gaussian mixture object */
  libRSF::GaussianComponent<2> Gaussian;
  libRSF::GaussianMixture<2> GMM;
  /** component 1 */
  Gaussian.setParamsCovariance(StdDev1.transpose()*StdDev1, Mean1, Weight1);
  GMM.addComponent(Gaussian);
  /** component 2*/
  Gaussian.setParamsCovariance(StdDev2.transpose()*StdDev2, Mean2, Weight2);
  GMM.addComponent(Gaussian);

  /** create zero-measurement */
  libRSF::Data AbsoluteMeasurement(libRSF::DataType::Point2, 0.0);
  AbsoluteMeasurement.setMean(libRSF::Vector2::Zero());

  /** create initial values */
  libRSF::Vector InitialValues = libRSF::Vector::LinSpaced(NumberPoints, -Range / 2, Range / 2);

  /** add state to graph */
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, 0.0);

  /** add factor to graph */
  if (ErrorModel == "Gaussian")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise);
  }
  else if (ErrorModel == "MaxMix")
  {
    libRSF::MaxMix2 MixtureNoiseMM(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseMM);
  }
  else if (ErrorModel == "SumMix")
  {
    libRSF::SumMix2 MixtureNoiseSM(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseSM);
  }
  else if (ErrorModel == "SumMixSpecial")
  {
    libRSF::SumMix2Special MixtureNoiseSMSpecial(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseSMSpecial);
  }
  else if (ErrorModel == "MaxSumMix")
  {
    libRSF::MaxSumMix2 MixtureNoiseMSM(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseMSM);
  }
  else if (ErrorModel == "DCS")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::DCSLoss(ScalingDCS));
  }
  else if (ErrorModel == "cDCE")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, NoiseIdentity, new libRSF::cDCELoss(StdDev1(0)));
  }
  else
  {
    PRINT_ERROR("Wrong error model: ", ErrorModel);
    return 1;
  }

  /** sample cost surface around the zero to get cost an gradient */
  SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).setMean(libRSF::Vector2::Zero());
  SimpleGraph.sampleCost2D(POSITION_STATE, 0.0, 0, NumberPoints, Range, CostSurfaceData);

  /** nested loop over points */
  for (int nPointX = 0; nPointX < NumberPoints; nPointX++)
  {
    for (int nPointY = 0; nPointY < NumberPoints; nPointY++)
    {
      /** construct initial value */
      libRSF::Vector2 Init;
      Init(0) = InitialValues(nPointX);
      Init(1) = InitialValues(nPointY);

      /** init state with value */
      SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).setMean(Init);

      /** save initial point */
      PreOptimizationData.addElement(POSITION_STATE, 0.0, SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0));

      /** optimize */
      SimpleGraph.solve(SolverOptions);

      /** save result */
      PostOptimizationData.addElement(POSITION_STATE, 0.0, SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0));

      /** save solution information */
      SolverData.addElement(SOLVE_TIME_STATE, libRSF::DataType::IterationSummary, 0.0);
      SolverData.getElement(SOLVE_TIME_STATE, 0.0, nPointX*NumberPoints+nPointY).setValueScalar(libRSF::DataElement::DurationSolver, SimpleGraph.getSolverDurationAndReset());
      SolverData.getElement(SOLVE_TIME_STATE, 0.0, nPointX*NumberPoints+nPointY).setValueScalar(libRSF::DataElement::IterationSolver, SimpleGraph.getSolverIterationsAndReset());

      libRSF::PrintProgress((100.0*nPointX)/NumberPoints);
    }
  }

  /** modify timestamps for identification */
  for (int nPoint = PostOptimizationData.countElement(POSITION_STATE, 0.0) - 1; nPoint >= 0; nPoint--)
  {
    PostOptimizationData.getElement(POSITION_STATE, 0.0, nPoint).setTimestamp(1.0);
  }

  return 0;
}

#ifndef TESTMODE // only compile main if not used in test context

int main(int argc, char** argv)
{
  /** init google logging for ceres */
  google::InitGoogleLogging(*argv);

  /** get command line arguments */
  std::vector<std::string> Arguments;
  libRSF::FactorGraphConfig Config;
  Config.ReadCommandLineOptions(argc, argv, &Arguments);

  /** datasets that store results*/
  libRSF::StateDataSet CostSurfaceData;
  libRSF::StateDataSet PreOptimizationData;
  libRSF::StateDataSet PostOptimizationData;
  libRSF::StateDataSet SolverData;

  if (CreateGraphAndSolve(Arguments,CostSurfaceData,PreOptimizationData,PostOptimizationData,SolverData) != 0)
  {
    return 1;
  }

  /** write everything to file */
  libRSF::WriteDataToFile(Config.OutputFile, "cost_gradient2", CostSurfaceData, false);
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, PreOptimizationData, true);
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, PostOptimizationData, true);
  libRSF::WriteDataToFile(Config.OutputFile, SOLVE_TIME_STATE, SolverData, true);

  return 0;
}

#endif // TESTMODE