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
* @file App_Robust_Models_1D.cpp
* @author Tim Pfeifer
* @date 26.08.2019
* @brief An simple application to evaluate the robust error functions for the scalar case.
* @copyright GNU Public License.
*
*/

#include "App_Robust_Models_1D.h"

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
  libRSF::Vector1 Mean1, Mean2, StdDev1, StdDev2, Weight1, Weight2;
  Mean1 << std::stod(Arguments.at(6));
  Mean2 << std::stod(Arguments.at(7));
  StdDev1 << std::stod(Arguments.at(8));
  StdDev2 << std::stod(Arguments.at(9));
  Weight1 << std::stod(Arguments.at(10));
  Weight2 << std::stod(Arguments.at(11));

  /** parameter of M-Estimators is the 10^Mean2 to allow a broad spread */
  const double MEstimatorParam = std::pow(10, Mean2(0));

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

  /** optional: check gradients */
//  SolverOptions.check_gradients = true;
//  SolverOptions.gradient_check_relative_precision = 1e-4;
//  SolverOptions.gradient_check_numeric_derivative_relative_step_size = 1e-3;

  /** optional: additional debugging */
//  std::vector<int> Iterations(100);
//  std::iota (std::begin(Iterations), std::end(Iterations), 1);
//  SolverOptions.trust_region_minimizer_iterations_to_dump = Iterations;
//  SolverOptions.trust_region_problem_dump_directory = ".";
//  SolverOptions.trust_region_problem_dump_format_type = ceres::DumpFormatType::TEXTFILE;

  /** configure Gaussian error model */
  libRSF::GaussianDiagonal<1> Noise;
  Noise.setStdDevDiagonal(StdDev1);

  /** configure Gaussian Identity model for cDCE */
  libRSF::GaussianDiagonal<1> NoiseIdentity;
  NoiseIdentity.setStdDevSharedDiagonal(1.0);

  /** create Gaussian mixture object */
  libRSF::GaussianComponent<1> Gaussian;
  libRSF::GaussianMixture<1> GMM;
  /** component 1 */
  Gaussian.setParamsStdDev(StdDev1, Mean1, Weight1);
  GMM.addComponent(Gaussian);
  /** component 2*/
  Gaussian.setParamsStdDev(StdDev2, Mean2, Weight2);
  GMM.addComponent(Gaussian);

  /** create zero-measurement */
  libRSF::Data AbsoluteMeasurement(libRSF::DataType::Point1, 0.0);
  AbsoluteMeasurement.setMean(libRSF::Vector1::Zero());

  /** create initial values */
  const libRSF::Vector InitialValues = libRSF::Vector::LinSpaced(NumberPoints, -Range / 2, Range / 2);

  /** add state to graph */
  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point1, 0.0);

  /** add factor to graph */
  if (ErrorModel == "Gaussian")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise);
  }
  else if (ErrorModel == "MaxMix")
  {
    libRSF::MaxMix1 MixtureNoiseMM(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseMM);
  }
  else if (ErrorModel == "SumMix")
  {
    libRSF::SumMix1 MixtureNoiseSM(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseSM);
  }
  else if (ErrorModel == "SumMixSpecial")
  {
    libRSF::SumMix1Special MixtureNoiseSMSpecial(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseSMSpecial);
  }
  else if (ErrorModel == "MaxSumMix")
  {
    libRSF::MaxSumMix1 MixtureNoiseMSM(GMM);
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, MixtureNoiseMSM);
  }
  else if (ErrorModel == "SC")
  {
    /**add switch variable */
    SimpleGraph.addState("Switch", libRSF::DataType::Switch, 0.0);
    libRSF::SwitchableConstraints<1, libRSF::GaussianDiagonal<1>> SC(Noise, MEstimatorParam);

    /** add factor*/
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0),
                                                      libRSF::StateID("Switch", 0.0, 0),
                                                      AbsoluteMeasurement, SC);
  }
  else if (ErrorModel == "DCS")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::DCSLoss(MEstimatorParam));
  }
  else if (ErrorModel == "DCE")
  {
    /**add covariance variable */
    SimpleGraph.addState("Covariance", libRSF::DataType::Covariance1, 0.0);

    /** set initial value */
    SimpleGraph.getStateData().getElement("Covariance", 0.0).setMean(StdDev1*StdDev1);

    /** set lower bound */
    SimpleGraph.setLowerBound("Covariance", 0.0, 0, StdDev1*StdDev1);

    /** create Dynamic Covariance Estimation error model */
    libRSF::DynamicCovarianceEstimation<1> DCE(StdDev1*StdDev1);

    /** add factor*/
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0),
                                                      libRSF::StateID("Covariance", 0.0, 0),
                                                      AbsoluteMeasurement, DCE);
  }
  else if (ErrorModel == "cDCE")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, NoiseIdentity, new libRSF::cDCELoss(StdDev1(0)));
  }
  else if (ErrorModel == "Student")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::StudentLoss(MEstimatorParam, 1));
  }
  else if (ErrorModel == "Huber")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::HuberLoss(MEstimatorParam));
  }
  else if (ErrorModel == "SoftLOne")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::SoftLOneLoss(MEstimatorParam));
  }
  else if (ErrorModel == "CauchyLoss")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::CauchyLoss(MEstimatorParam));
  }
  else if (ErrorModel == "CauchyPDFLoss")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::CauchyPDFLoss(MEstimatorParam));
  }
  else if (ErrorModel == "TukeyLoss")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::TukeyLoss(MEstimatorParam));
  }
  else if (ErrorModel == "GeneralAdaptiveLoss")
  {
    SimpleGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(POSITION_STATE, 0.0, 0), AbsoluteMeasurement, Noise, new libRSF::GeneralAdaptiveLoss(MEstimatorParam));
  }
  else
  {
    PRINT_ERROR("Wrong error model: ", ErrorModel);
    return 1;
  }

  /** sample cost surface around the zero to get cost an gradient */
  SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).setMean(libRSF::Vector1::Zero());
  if (ErrorModel == "SC" || ErrorModel == "DCE")
  {
    SimpleGraph.sampleCost1D(POSITION_STATE, 0.0, 0, NumberPoints, Range, CostSurfaceData, true);
  }
  else
  {
    SimpleGraph.sampleCost1D(POSITION_STATE, 0.0, 0, NumberPoints, Range, CostSurfaceData, false);
  }

  /** loop over points */
  for (int nPoint = 0; nPoint < NumberPoints; nPoint++)
  {
    /** init state with value */
    SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0).setMean(InitialValues.segment(nPoint, 1));

    /** save initial point */
    PreOptimizationData.addElement(POSITION_STATE, 0.0, SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0));

    /** optimize */
    SimpleGraph.solve(SolverOptions);

    /** save result */
    PostOptimizationData.addElement(POSITION_STATE, 0.0, SimpleGraph.getStateData().getElement(POSITION_STATE, 0.0));

    /** save solution information */
    SolverData.addElement(SOLVE_TIME_STATE, libRSF::DataType::IterationSummary, 0.0);
    SolverData.getElement(SOLVE_TIME_STATE, 0.0, nPoint).setValueScalar(libRSF::DataElement::DurationSolver, SimpleGraph.getSolverDurationAndReset());
    SolverData.getElement(SOLVE_TIME_STATE, 0.0, nPoint).setValueScalar(libRSF::DataElement::IterationSolver, SimpleGraph.getSolverIterationsAndReset());

    libRSF::PrintProgress((100.0*nPoint)/NumberPoints);
  }

  /** modify timestamps for identification */
  for (int nPoint = NumberPoints - 1; nPoint >= 0; nPoint--)
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
  libRSF::WriteDataToFile(Config.OutputFile, "cost_gradient1", CostSurfaceData, false);
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, PreOptimizationData, true);
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, PostOptimizationData, true);
  libRSF::WriteDataToFile(Config.OutputFile, SOLVE_TIME_STATE, SolverData, true);

  return 0;
}

#endif // TESTMODE
