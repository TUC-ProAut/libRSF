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

#include "Example_Adaptive_2D.h"

void AddRange(libRSF::FactorGraph &Graph,
              const libRSF::Data &Range,
              const double TimePosition)
{
  /** create initial GMM with one component*/
  libRSF::GaussianMixture<1> GMM;
  GMM.addDiagonal(libRSF::Vector1::Zero(), Range.getStdDevDiagonal(), libRSF::Vector1::Ones());
  libRSF::MaxSumMix1 MixtureNoise(GMM);

  /** add factor */
  Graph.addFactor<libRSF::FactorType::Range2>(libRSF::StateID(POSITION_STATE, TimePosition), Range, MixtureNoise);
}

void SaveResult(libRSF::FactorGraph &Graph,
                libRSF::StateDataSet &Result,
                const double Offset)
{
  /** export position */
  std::vector<libRSF::Data> Positions = Graph.getStateData().getElementsOfID(POSITION_STATE);
  for (libRSF::Data &Pos : Positions)
  {
    Pos.setTimestamp(Pos.getTimestamp() + Offset);
    Result.addElement(POSITION_STATE, Pos);
  }
}


int main(int ArgC, char** ArgV)
{
  google::InitGoogleLogging(*ArgV);

  /** process command line parameter */
  libRSF::FactorGraphConfig Config;
  if (!Config.ReadCommandLineOptions(ArgC, ArgV))
  {
    PRINT_ERROR("Reading configuration gone wrong! Exit now!");
    return 0;
  }

  /** read input data */
  libRSF::SensorDataSet Measurements;
  libRSF::ReadDataFromFile(Config.InputFile, Measurements);

  /** find first time stamp */
  double TimeFirst;
  if (!Measurements.getTimeFirstOverall(TimeFirst))
  {
    PRINT_ERROR("Dataset is empty!");
    return 1;
  }

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;

  /** create prior noise models */
  libRSF::GaussianDiagonal<2> NoisePriorPoint;
  NoisePriorPoint.setStdDevSharedDiagonal(100.0);

  libRSF::GaussianDiagonal<1> NoisePriorAngle;
  NoisePriorAngle.setStdDevSharedDiagonal(2 * M_PI);

  /** create prior measurements */
  libRSF::Data PriorPoint(libRSF::DataType::Point2, TimeFirst);
  libRSF::Data PriorAngle(libRSF::DataType::Angle, TimeFirst);
  PriorPoint.setMean(libRSF::Vector2::Zero());
  PriorAngle.setMean(libRSF::Vector1::Zero());

  /** add first states */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeFirst);

  /** add prior */
  Graph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, TimeFirst), PriorPoint, NoisePriorPoint);
  Graph.addFactor<libRSF::FactorType::PriorAngle>(libRSF::StateID(ORIENTATION_STATE, TimeFirst), PriorAngle, NoisePriorAngle);

  /** loop over measurements */
  double TimeNow = TimeFirst;
  double TimePrev = TimeFirst;
  do
  {
    /** add new states and odometry */
    if (TimeNow != TimeFirst)
    {
      /** new states */
      Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeNow);
      Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeNow);

      /** get odometry measurement */
      double TimeOdom = 0.0;
      if (!Measurements.getTimeBelowOrEqual(libRSF::DataType::Odom2, TimeNow, TimeOdom))
      {
        PRINT_ERROR("Could not find measurement below: ", TimeNow);
        return 0;
      }
      const libRSF::Data Odom = Measurements.getElement(libRSF::DataType::Odom2, TimeOdom);

      /** create noise model */
      libRSF::GaussianDiagonal<3> GaussianOdom;
      GaussianOdom.setCovarianceDiagonal(Odom.getCovarianceDiagonal());

      /** add odometry factor */
      Graph.addFactor<libRSF::FactorType::Odom2>(libRSF::StateID(POSITION_STATE, TimePrev),
                                                 libRSF::StateID(ORIENTATION_STATE, TimePrev),
                                                 libRSF::StateID(POSITION_STATE, TimeNow),
                                                 libRSF::StateID(ORIENTATION_STATE, TimeNow),
                                                 Odom, GaussianOdom);
    }

    /** get range measurements */
    const std::vector<libRSF::Data> Ranges = Measurements.getElements(libRSF::DataType::Range2, TimeNow);

    /** add range measurements */
    for (const libRSF::Data &Range: Ranges)
    {
      AddRange(Graph, Range, TimeNow);
    }

    TimePrev = TimeNow;
  }
  while (Measurements.getTimeNext(libRSF::DataType::Range2, TimePrev, TimeNow));

  /** create initial GMM*/
  libRSF::GaussianMixture<1> GMM;

  /** create density estimation config */
  libRSF::GaussianMixture<1>::EstimationConfig GMMConfig = libRSF::GaussianMixture<1>::ConvertConfig(Config.Ranging.ErrorModel.GMM);
  GMMConfig.RemoveSmallComponents = true;

  /** solve the graph initially */
  Graph.solve(Config.SolverConfig);
  SaveResult(Graph, Result, 0);

  const int NumAdapt = 10;
  for (int nAdapt = 1; nAdapt <= NumAdapt; nAdapt++)
  {
    PRINT_LOGGING("Starting iteration ", nAdapt, " of ", NumAdapt);

    /** get residuals */
    libRSF::Matrix Residuals;
    Graph.computeUnweightedErrorMatrix(libRSF::FactorType::Range2, Residuals);

    /** calculate statistics for GMM initialization*/
    const libRSF::Vector1 Mean = -Residuals.rowwise().mean();
    const libRSF::Vector1 Covariance = libRSF::EstimateSampleCovariance<1>(Residuals);
    const libRSF::Vector1 Weight = libRSF::Vector1::Ones() / (GMM.getNumberOfComponents() + 1);

    /** add new component */
    libRSF::GaussianComponent<1> Component;
    Component.setParamsCovariance(Covariance, Mean, Weight);
    GMM.addComponent(Component);

    /** adapt error model */
    const int CompBefore = GMM.getNumberOfComponents();
    GMM.estimate(Residuals, GMMConfig);
    const int CompAfter = GMM.getNumberOfComponents();
    const libRSF::MaxSumMix1 MixtureNoise(GMM);
    Graph.setNewErrorModel(libRSF::FactorType::Range2, MixtureNoise);

    /** solve the graph with the new model */
    Graph.solve(Config.SolverConfig);

    /** define time offset to distinguish iterations*/
    const double TimeOffset = nAdapt*100000;

    /** export position to result set*/
    SaveResult(Graph, Result, TimeOffset);

    /** export GMM to result set */
    const libRSF::Data GMMData = GMM.exportToStateData(TimeOffset);
    Result.addElement(GMM_RESULT, GMMData);

    /** export residuals to result set */
    for (int n = 0; n < Residuals.cols(); n++)
    {
      libRSF::Data ResidualData(libRSF::DataType::Error1, TimeOffset);
      ResidualData.setMean(Residuals.col(n));
      Result.addElement(RESIDUAL_RESULT, ResidualData);
    }

    /** check for convergence */
    if (CompAfter < CompBefore)
    {
      break;
    }
  }

  /** export all results to file */
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result, false);
  libRSF::WriteDataToFile(Config.OutputFile, GMM_RESULT, Result, true);
  libRSF::WriteDataToFile(Config.OutputFile, RESIDUAL_RESULT, Result, true);

  return 0;
}
