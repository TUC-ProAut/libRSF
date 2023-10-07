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

#include "App_SLAM_BR_2D.h"

void InitGraph(libRSF::FactorGraph &Graph,
               const double TimeInitial)
{
  /** set prior vectors */
  const libRSF::Vector1 AngleVect = libRSF::Vector1::Zero();
  const libRSF::Vector2 PositionVect = libRSF::Vector2::Zero();

  /** create prior noise models */
  libRSF::GaussianDiagonal<2> NoisePriorPoint;
  NoisePriorPoint.setCovarianceDiagonal(libRSF::Vector2::Ones()*0.1);

  libRSF::GaussianDiagonal<1> NoisePriorAngle;
  NoisePriorAngle.setCovarianceDiagonal(libRSF::Vector1::Ones()*5*M_PI/180);

  /** create prior measurements */
  libRSF::Data PriorPoint(libRSF::DataType::Point2, TimeInitial);
  libRSF::Data PriorAngle(libRSF::DataType::Angle, TimeInitial);
  PriorPoint.setMean(PositionVect);
  PriorAngle.setMean(AngleVect);

  /** add first states */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeInitial);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeInitial);

  /** add prior */
  Graph.addFactor<libRSF::FactorType::Prior2>(libRSF::StateID(POSITION_STATE, TimeInitial), PriorPoint, NoisePriorPoint);
  Graph.addFactor<libRSF::FactorType::PriorAngle>(libRSF::StateID(ORIENTATION_STATE, TimeInitial), PriorAngle, NoisePriorAngle);
}

void AddOdometry(libRSF::FactorGraph &Graph,
                 libRSF::SensorDataSet &Measurements,
                 const double TimeOld,
                 const double TimeNow)
{
  /** add states */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimeNow);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimeNow);

  /** get measurement */
  const libRSF::Data Pose = Measurements.getElement(libRSF::DataType::PoseBetween2, TimeNow);

  /** create noise model */
  libRSF::GaussianFull<3> Noise;
  Noise.setCovarianceMatrix(Pose.getCovarianceMatrix());

  /** add factor */
  Graph.addFactor<libRSF::FactorType::BetweenPose2>(libRSF::StateID(POSITION_STATE, TimeOld),
                                                    libRSF::StateID(ORIENTATION_STATE, TimeOld),
                                                    libRSF::StateID(POSITION_STATE, TimeNow),
                                                    libRSF::StateID(ORIENTATION_STATE, TimeNow),
                                                    Pose,
                                                    Noise);
}

bool AddLandmarks(std::vector<std::string> &LandmarkStrings,
                  libRSF::FactorGraph &Graph,
                  const libRSF::SensorDataSet &Measurements,
                  const double TimeNow)
{
  if (Measurements.checkElement(libRSF::DataType::BearingRangeID2, TimeNow))
  {
    /** get available measurements */
    const std::vector<libRSF::Data> BearingRanges = Measurements.getElements(libRSF::DataType::BearingRangeID2, TimeNow);

    /** sort measurements regarding ID */
    std::multimap<int, libRSF::Data> SortedBR;
    for (const libRSF::Data &BR : BearingRanges)
    {
      SortedBR.emplace(BR.getValue(libRSF::DataElement::ID)(0), BR);
    }

    /** loop over IDs */
    for (auto It = SortedBR.begin(); It != SortedBR.end(); It = SortedBR.upper_bound(It->first))
    {
      const int ID = It->first;
      const int NumBRs = SortedBR.count(ID);

      /** create identifying string for landmark */
      const std::string LM_ID_STATE = LANDMARK_STATE + std::to_string(ID);

      /** get data from measurements*/
      libRSF::Matrix Means(NumBRs, 2);
      libRSF::Matrix Covariances(NumBRs, 2);
      auto ItBR = It;
      for (int n = 0; n < NumBRs; n++)
      {
        Means.row(n) = ItBR->second.getMean();
        Covariances.row(n) = ItBR->second.getCovarianceDiagonal();

        /** iterate to next measurement */
        ItBR++;
      }

      /** create mean measurement */
      const libRSF::Vector2 MeanBR = Means.colwise().mean();

      libRSF::Data MeanMeasurement(libRSF::DataType::BearingRangeID2, TimeNow);
      MeanMeasurement.setMean(MeanBR);

      /** assume equal weights */
      const libRSF::Vector1 Weight = libRSF::Vector1::Ones() / NumBRs;

      /** create Gaussian mixture model */
      libRSF::GaussianMixture<2> GMM;
      for (int n = 0; n < NumBRs; n++)
      {
        /** correct mean = mean of measurements - measurement
        (we flip the sign because a noise model describes the error, not the measurement) */
        libRSF::Vector2 MeanBRCorr =  -(Means.row(n).transpose() - MeanBR);
        MeanBRCorr(0) = libRSF::NormalizeAngle(MeanBRCorr(0));

        /** create a Gaussian component */
        libRSF::GaussianComponent<2> Comp;
        Comp.setParamsCovariance(Covariances.row(n).asDiagonal(), MeanBRCorr, Weight);

        GMM.addComponent(Comp);
      }

      /** check if landmark exist --> initialize if not*/
      bool DoPrediction = false;
      if (!Graph.getStateData().checkID(LM_ID_STATE))
      {
        /** store new string */
        LandmarkStrings.emplace_back(LM_ID_STATE);

        /** add state to graph */
        Graph.getStateData().addElement(LM_ID_STATE, libRSF::DataType::PointID2, TimeNow);

        /** set ID of landmark */
        Graph.getStateData().getElement(LM_ID_STATE, TimeNow).setValueScalar(libRSF::DataElement::ID, ID);

        /** use predict function of BR factor to initialize */
        DoPrediction = true;
      }

      /** get landmark time */
      double TimeLM = 0.0;
      Graph.getStateData().getTimeFirst(LM_ID_STATE, TimeLM);

      /** put GMM into least squares representation */
      libRSF::MaxSumMix2 MixtureNoise(GMM);

      /** add factors */
      Graph.addFactor<libRSF::FactorType::BetweenBearingRange2>(libRSF::StateID(POSITION_STATE, TimeNow),
                                                                libRSF::StateID(ORIENTATION_STATE, TimeNow),
                                                                libRSF::StateID(LM_ID_STATE, TimeLM),
                                                                MeanMeasurement,
                                                                MixtureNoise,
                                                                DoPrediction);
    }

    /** at least one measurement was added */
    return true;
  }
  else
  {
    /** no measurement was added */
    return false;
  }
}

int main(int ArgC, char** ArgV)
{
  /** verbose logging for ceres */
  google::InitGoogleLogging(*ArgV);
  google::LogToStderr();
  google::SetStderrLogging(google::GLOG_INFO);

  /** process command line parameter */
  libRSF::FactorGraphConfig Config;
  if(!Config.ReadCommandLineOptions(ArgC, ArgV))
  {
    PRINT_ERROR("Reading configuration gone wrong! Exit now!");
    return 0;
  }

  /** variable to keep track the existing IDs for landmark variables */
  std::vector<std::string> LandmarkStrings;

  /** read input data */
  libRSF::SensorDataSet Measurements;
  libRSF::ReadDataFromFile(Config.InputFile, Measurements);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateDataSet Result;

  /** to store the duration of different steps */
  libRSF::Data Summary(libRSF::DataType::IterationSummary, 0.0);

  /** find time limits of odometry */
  double TimeFirst = 0.0, TimeLast = 0.0, TimeNow = 0.0, TimeOld;
  Measurements.getTimeFirst(libRSF::DataType::PoseBetween2, TimeFirst);
  Measurements.getTimeLast(libRSF::DataType::PoseBetween2, TimeLast);

  /** find first timestamp before odometry */
  const libRSF::Data FirstOdom = Measurements.getElement(libRSF::DataType::PoseBetween2, TimeFirst);
  const double TimeBeforeFirst = FirstOdom.getValue(libRSF::DataElement::TimestampRef)(0);

  /** initial graph with a first set of states */
  InitGraph(Graph, TimeBeforeFirst);
  Summary = libRSF::Data(libRSF::DataType::IterationSummary, TimeBeforeFirst);
  Save(Graph, Config, Summary, Result, false);

  /** loop over odometry */
  TimeOld = TimeBeforeFirst;
  TimeNow = TimeFirst;
  libRSF::Timer IterationTimer;
  do
  {
    /** update current timestamp and reset duration */
    Summary = libRSF::Data(libRSF::DataType::IterationSummary, TimeNow);

    /** start timer*/
    IterationTimer.reset();

    /** add odometry */
    AddOdometry(Graph, Measurements, TimeOld, TimeNow);

    /** add bearing range measurements*/
    if (AddLandmarks(LandmarkStrings, Graph, Measurements, TimeNow))
    {
      /** we do a full optimization run if there is any landmark */
      Graph.setAllVariable();
    }

    /** solve graph, force solve every 60 seconds */
    Solve(Graph, Config, Summary, fmod(TimeNow, 60.0) < (TimeNow - TimeOld) * 1.1);

    /** save iteration timestamp */
    Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());

    /** save result */
    Save(Graph, Config, Summary, Result, false);

    /** print progress every 10%*/
    libRSF::PrintProgress((TimeNow - TimeFirst) / (TimeLast - TimeFirst) * 100);

    /** increment time */
    TimeOld = TimeNow;
  }
  while (Measurements.getTimeNext(libRSF::DataType::PoseBetween2, TimeOld, TimeNow));

  /** calculate and save final solution*/
  Summary.setTimestamp(TimeNow);
  Solve(Graph, Config, Summary, true);
  Summary.setValueScalar(libRSF::DataElement::DurationTotal, IterationTimer.getSeconds());
  Save(Graph, Config, Summary, Result, true);

  /** print last report */
  Graph.printReport();

  /** export ego position to file */
  libRSF::WriteDataToFile(Config.OutputFile, POSITION_STATE, Result, false);
  libRSF::WriteDataToFile(Config.OutputFile, ORIENTATION_STATE, Result, true);

  /** export landmark positions to file (and compute covariance if required) */
  if(Config.Solution.EstimateCov)
  {
    /** fixed states can bias covariance estimation, so make all variable */
    Graph.setAllVariable();
  }
  for (const string &ID : LandmarkStrings)
  {
    if(Config.Solution.EstimateCov)
    {
      Graph.computeCovariance(ID);
    }
    libRSF::WriteDataToFile(Config.OutputFile, ID, Graph.getStateData(), true);
  }

  /** additional info regarding runtime and iterations */
  libRSF::WriteDataToFile(Config.OutputFile, SOLVE_TIME_STATE, Result, true);

  return 0;
}
