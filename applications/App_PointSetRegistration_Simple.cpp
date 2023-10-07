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

#include "App_PointSetRegistration.h"

void ConcatPointsToGMM(const std::vector<libRSF::Data> &Points, libRSF::GaussianMixture<2> &GMM)
{
  /** weight */
  libRSF::Vector1 Weight;
  Weight(0) = 1.0 / static_cast<double>(Points.size() + 1);

  for(const libRSF::Data &Point: Points)
  {
    /** mean */
    libRSF::Vector2 Mean = Point.getMean();

    /** uncertainty */
    libRSF::Matrix22 Cov = Point.getCovarianceMatrix();

    /** add component */
    libRSF::GaussianComponent<2> Gauss;
    Gauss.setParamsCovariance(Cov, Mean, Weight);
    GMM.addComponent(Gauss);

  }
}

void AddPointSetRegistration(libRSF::FactorGraph &Graph,
                             libRSF::GaussianMixture<2> &GMM,
                             const libRSF::FactorGraphConfig &Config,
                             const std::vector<libRSF::Data> &PointsOld,
                             const std::vector<libRSF::Data> &PointsNew,
                             const double TimeOld,
                             const double TimeNew)
{
    /** build error model out of old point set */
    GMM.clear();
    ConcatPointsToGMM(PointsOld, GMM);

    /** loop over points */
    for(const libRSF::Data &Point: PointsNew)
    {
      /** create point correspondence */
      libRSF::Data PointSet(libRSF::DataType::Point2Set, TimeNew);
      PointSet.setMean((libRSF::Vector4() << 0, 0, Point.getMean()).finished());

      /** inflate error model */
      libRSF::GaussianMixture<2> GMMD2D = GMM;
      GMMD2D.inflateWithCov(Point.getCovarianceMatrix());

      /** add factors */
      switch (Config.Radar.ErrorModel.GMM.MixtureType)
      {
        case libRSF::ErrorModelMixtureType::MaxMix:
        {
          libRSF::MaxMix2 Mixture(GMMD2D);
          Graph.addFactor<libRSF::FactorType::Point2RegPose>(
                  libRSF::StateID(POSE_STATE, TimeOld, 0),
                  libRSF::StateID(POSE_STATE, TimeNew, 0),
                  PointSet,
                  Mixture);
        }
        break;

        case libRSF::ErrorModelMixtureType::SumMix:
        {
          libRSF::SumMix2 Mixture(GMMD2D);
          Graph.addFactor<libRSF::FactorType::Point2RegPose>(
                  libRSF::StateID(POSE_STATE, TimeOld, 0),
                  libRSF::StateID(POSE_STATE, TimeNew, 0),
                  PointSet,
                  Mixture);
        }
        break;

        case libRSF::ErrorModelMixtureType::MaxSumMix:
        {
          libRSF::MaxSumMix2 Mixture(GMMD2D);
          Graph.addFactor<libRSF::FactorType::Point2RegPose>(
                  libRSF::StateID(POSE_STATE, TimeOld, 0),
                  libRSF::StateID(POSE_STATE, TimeNew, 0),
                  PointSet,
                  Mixture);
        }
        break;

      default:
        PRINT_ERROR("Wrong error model!");
          break;
      }
    }
}

void UpdatePointSetErrorModel (libRSF::FactorGraph &Graph,
                               const libRSF::GaussianMixture<2> &GMM,
                               const libRSF::FactorGraphConfig &Config,
                               const std::vector<libRSF::Data> &PointsOld,
                               const std::vector<libRSF::Data> &PointsNew,
                               const double TimeOld,
                               const double TimeNew)
{

    /** correct error model with rotated cov */
    libRSF::Rotation2D Rot12(Graph.getStateData().getElement(POSE_STATE, TimeNew,0).getMean()(2)
                             - Graph.getStateData().getElement(POSE_STATE, TimeOld,0).getMean()(2));
    int nPoint = 0;
    for(const libRSF::Data &Point: PointsNew)
    {
      /** calculate rotated matrix */
      libRSF::Matrix22 Cov = Rot12.toRotationMatrix()*Point.getCovarianceMatrix()*Rot12.toRotationMatrix().transpose();

      /** create distribution to distribution model */
      libRSF::GaussianMixture<2> GMMD2D = GMM;
      GMMD2D.inflateWithCov(Cov);

      /** replace error model */
      switch (Config.Radar.ErrorModel.GMM.MixtureType)
      {
        case libRSF::ErrorModelMixtureType::MaxMix:
        {
          libRSF::MaxMix2 Mixture(GMMD2D);
          Graph.setNewErrorModel(libRSF::FactorType::Point2RegPose, TimeOld, nPoint, Mixture);
        }
        break;

        case libRSF::ErrorModelMixtureType::SumMix:
        {
          libRSF::SumMix2 Mixture(GMMD2D);
          Graph.setNewErrorModel(libRSF::FactorType::Point2RegPose, TimeOld, nPoint, Mixture);
        }
        break;

        case libRSF::ErrorModelMixtureType::MaxSumMix:
        {
          libRSF::MaxSumMix2 Mixture(GMMD2D);
          Graph.setNewErrorModel(libRSF::FactorType::Point2RegPose, TimeOld, nPoint, Mixture);
        }
        break;

      default:
        PRINT_ERROR("Wrong error model!");
          break;
      }

      nPoint++;
    }
}


int main(int ArgC, char** ArgV)
{
  /** init logging */
  google::InitGoogleLogging(*ArgV);

  /** process command line parameter */
  libRSF::FactorGraphConfig Config;
  if(!Config.ReadCommandLineOptions(ArgC, ArgV))
  {
    PRINT_ERROR("Reading configuration gone wrong! Exit now!");
    return 0;
  }

  /** read input data */
  libRSF::SensorDataSet Measurements;
  libRSF::ReadDataFromFile(Config.InputFile, Measurements);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;

  /** configure solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.max_num_iterations = 1000;
  SolverOptions.max_solver_time_in_seconds = 30;
  SolverOptions.num_threads = 4;

  /** get first timestamp */
  double TimeNew, TimeOld;
  if(!Measurements.getTimeFirst(libRSF::DataType::Point2, TimeNew))
  {
    PRINT_ERROR("There is no point measurement!");
    return 0;
  }
  TimeOld = TimeNew;

  /** add first states to graph */
  Graph.addState(POSE_STATE,libRSF::DataType::Pose2, TimeNew);

  /** get first set of points */
  std::vector<libRSF::Data> PointsNew, PointsOld;
  PointsOld = Measurements.getElements(libRSF::DataType::Point2, TimeNew);

  /** loop over timestamps */
  while(Measurements.getTimeNext(libRSF::DataType::Point2, TimeOld, TimeNew))
  {
    /** add states to graph */
    Graph.addState(POSE_STATE, libRSF::DataType::Pose2, TimeNew);

    /** copy old pose */
    Graph.getStateData().getElement(POSE_STATE, TimeNew).setMean(Graph.getStateData().getElement(POSE_STATE, TimeOld).getMean());

    /** freeze old states */
    Graph.setConstant(POSE_STATE, TimeOld);

    /** get new point set */
    PointsNew = Measurements.getElements(libRSF::DataType::Point2, TimeNew);

    /** add factors */
    libRSF::GaussianMixture<2> GMM12;
    libRSF::GaussianMixture<2> GMM21;
    AddPointSetRegistration(Graph, GMM12, Config, PointsOld, PointsNew, TimeOld, TimeNew);

    /** solve robust */
    Graph.solve(SolverOptions);

    /** update error model with correct rotation */
    UpdatePointSetErrorModel(Graph, GMM12, Config, PointsOld, PointsNew, TimeOld, TimeNew);

    /** solve robust with rotated covariance */
    Graph.solve(SolverOptions);

    /** compute covariance */
    if(Config.Solution.EstimateCov)
    {
      Graph.computeCovarianceSigmaPoints(POSE_STATE, TimeNew);

      /** apply scaling if double sided */
      Graph.getStateData().getElement(POSE_STATE, TimeNew).setCovariance(Graph.getStateData().getElement(POSE_STATE, TimeNew).getCovarianceMatrix());
    }

    PointsOld = PointsNew;
    TimeOld = TimeNew;
  }

  /** export data to file */
  libRSF::WriteDataToFile(Config.OutputFile, POSE_STATE, Graph.getStateData(), false);

  return 0;
}
