/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option); any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de);
 ***************************************************************************/

#include "AppPool_Init.h"

void PseudoRangeRANSAC(libRSF::FactorGraph &Graph,
                       const libRSF::FactorGraphConfig &Config,
                       const libRSF::SensorDataSet &Measurements,
                       const double TimeStart,
                       const double TimeEnd,
                       const int Iterations,
                       const double Threshold)
{
  /** find measurements */
  std::vector<double> Times;
  if (!Measurements.getTimesBetween(libRSF::DataType::Pseudorange3, TimeStart, TimeEnd, Times))
  {
    PRINT_WARNING("No pseudorange measurements available between ", TimeStart, " and ", TimeEnd, "!");
  }
  else
  {
    /** configure the solver */
    ceres::Solver::Options SolverOptions;
    SolverOptions.minimizer_progress_to_stdout = false;
    SolverOptions.use_nonmonotonic_steps = true;
    SolverOptions.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
    SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
    SolverOptions.max_num_iterations = 15;
    SolverOptions.num_threads = 1;

    /** create uncertainty for priors */
    libRSF::GaussianDiagonal<3> GaussianPriorPos;
    GaussianPriorPos.setStdDevSharedDiagonal(30);
    libRSF::GaussianDiagonal<1> GaussianPriorClock;
    GaussianPriorClock.setStdDevSharedDiagonal(30);

    /** set up random generator */
    std::mt19937 Generator{std::random_device{}()};

    /** repeat RANSAC for each timestamp independently */
    for (const double Time : Times)
    {
      libRSF::Vector3 OldPosition;
      libRSF::Vector1 OldClock;

      /** get existing state for prior and initialization */
      double TimePosition, TimeClock;
      bool HasState;
      if (Graph.getStateData().getTimeBelowOrEqual(POSITION_STATE, Time, TimePosition) &&
          Graph.getStateData().getTimeBelowOrEqual(POSITION_STATE, Time, TimeClock))
      {
        HasState = true;
        OldPosition = Graph.getStateData().getElement(POSITION_STATE, TimePosition, 0).getMean();
        OldClock = Graph.getStateData().getElement(CLOCK_ERROR_STATE, TimeClock, 0).getMean();
      }
      else
      {
        OldPosition.setZero();
        OldClock.setZero();
      }

      /** get current set of GNSS measurements */
      const std::vector<libRSF::Data> PseudoRanges = Measurements.getElements(libRSF::DataType::Pseudorange3, Time);
      const int DataSize = static_cast<int>(PseudoRanges.size());

      /** create full graph */
      libRSF::FactorGraph FullGraph;
      FullGraph.addState(POSITION_STATE, libRSF::DataType::Point3, Time);
      FullGraph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, Time);
      for (const libRSF::Data &PR : PseudoRanges)
      {
        AddPseudorange3(FullGraph, Config, PR,Time);
      }

      /** find another initial guess if the current one is zero (not initialized)*/
      if (OldPosition.isZero())
      {
        FullGraph.solve(SolverOptions);
        OldPosition = FullGraph.getStateData().getElement(POSITION_STATE, Time, 0).getMean();
        OldClock = FullGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).getMean();
      }

      /** create measurements for prior */
      libRSF::Data PriorPos(libRSF::DataType::Point3, Time);
      PriorPos.setMean(OldPosition);
      libRSF::Data PriorClock(libRSF::DataType::ClockError, Time);
      PriorClock.setMean(OldClock);

      /** set up partial optimization problem */
      libRSF::FactorGraph PartialGraph;
      PartialGraph.addState(POSITION_STATE, libRSF::DataType::Point3, Time);
      PartialGraph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, Time);
      PartialGraph.addFactor<libRSF::FactorType::Prior3>(libRSF::StateID(POSITION_STATE, Time, 0), PriorPos, GaussianPriorPos);
      PartialGraph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(CLOCK_ERROR_STATE, Time, 0), PriorClock, GaussianPriorClock);

      /** prepare index vector */
      std::vector<int> Index;
      Index.resize(DataSize);
      std::iota(std::begin(Index), std::end(Index), 0);

      /** RANSAC loop */
      double BestConsensus = 0;
      libRSF::Vector3 BestPosition = OldPosition;
      libRSF::Vector1 BestClock = OldClock;
      for (int n = 0; n < Iterations; n++)
      {
        /** create random order */
        std::shuffle(std::begin(Index), std::end(Index), Generator);

        /** add random set of 4 pseudo ranges */
        for (int m = 0; m < std::min(DataSize, 4); m++)
        {
          AddPseudorange3(PartialGraph, Config, PseudoRanges.at(Index.at(m)), Time);
        }

        /** use old state as initialization */
        PartialGraph.getStateData().getElement(POSITION_STATE, Time, 0).setMean(OldPosition);
        PartialGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).setMean(OldClock);

        /** solve */
        PartialGraph.solve(SolverOptions);

        /** clean up */
        PartialGraph.removeFactor(Config.GNSS.Type, Time);

        /** apply the new estimation to the full graph*/
        FullGraph.getStateData().getElement(POSITION_STATE, Time, 0).setMean(PartialGraph.getStateData().getElement(POSITION_STATE, Time, 0).getMean());
        FullGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).setMean(PartialGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).getMean());

        /** get residual for all measurements from the full graph */
        std::vector<double> CurrentResiduals;
        FullGraph.computeUnweightedError(Config.GNSS.Type, CurrentResiduals);

        /** evaluate true consensus */
        double CurrentConsensus = 0;
        for (const double Res : CurrentResiduals)
        {
          /** normalize residual */
          if (std::abs(Res) < Threshold)
          {
            /** RANSAC */
//            CurrentConsensus += 1.0;

            /** MSAC */
            CurrentConsensus += 1 - std::abs(Res) / Threshold;
          }
        }
        /** store best result */
        if (CurrentConsensus > BestConsensus)
        {
          BestConsensus = CurrentConsensus;
          BestPosition = PartialGraph.getStateData().getElement(POSITION_STATE, Time, 0).getMean();
          BestClock = PartialGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).getMean();
        }
      }

      /** refine the best result using the whole consensus set*/
      std::vector<double> BestResiduals;
      FullGraph.getStateData().getElement(POSITION_STATE, Time, 0).setMean(BestPosition);
      FullGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).setMean(BestClock);
      FullGraph.computeUnweightedError(Config.GNSS.Type, BestResiduals);
      for (int m = 0; m < static_cast<int>(BestResiduals.size()); m++)
      {
        if (std::abs(BestResiduals.at(m)) < Threshold)
        {
          AddPseudorange3(PartialGraph, Config, PseudoRanges.at(m), Time);
        }
      }
      PartialGraph.solve();
      BestPosition = PartialGraph.getStateData().getElement(POSITION_STATE, Time, 0).getMean();
      BestClock = PartialGraph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).getMean();

      /** copy best result */
      if (HasState)
      {
        Graph.getStateData().getElement(POSITION_STATE, TimePosition, 0).setMean(BestPosition);
        Graph.getStateData().getElement(CLOCK_ERROR_STATE, TimeClock, 0).setMean(BestClock);
      }
      else
      {
        PRINT_WARNING("There is no position state, add one!");
        Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, Time);
        Graph.addStateWithCheck(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, Time);
        Graph.getStateData().getElement(POSITION_STATE, Time, 0).setMean(BestPosition);
        Graph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).setMean(BestClock);
      }
    }
  }
}

void PseudoRangeSampling(libRSF::FactorGraph &Graph,
                         const double Time,
                         const int Iterations,
                         const double SamplingStdDev)
{
  /** save current state */
  libRSF::Vector3 Position = Graph.getStateData().getElement(POSITION_STATE, Time, 0).getMean();
  libRSF::Vector1 Clock = Graph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).getMean();

  /** save current cost */
  double Cost = Graph.getCost();

  /** setup random number generator */
  std::default_random_engine Generator;
  std::normal_distribution<double> GaussianX(Position(0), SamplingStdDev);
  std::normal_distribution<double> GaussianY(Position(1), SamplingStdDev);
  std::normal_distribution<double> GaussianZ(Position(2), SamplingStdDev);
  std::normal_distribution<double> GaussianClock(Clock(0), SamplingStdDev);

  /** iterate over samples */
  libRSF::Vector3 BestPosition = Position;
  libRSF::Vector1 BestClock = Clock;
  for (int n = 0; n < Iterations; n++)
  {
    /** draw samples */
    Position(0) = GaussianX(Generator);
    Position(1) = GaussianY(Generator);
    Position(2) = GaussianZ(Generator);
    Clock(0) = GaussianClock(Generator);

    /** change states */
    Graph.getStateData().getElement(POSITION_STATE, Time, 0).setMean(Position);
    Graph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).setMean(Clock);

    /** evaluate the cost of this sample */
    const double SampleCost = Graph.getCost();

    /** save sample if it is better */
    if (SampleCost < Cost)
    {
      Cost = SampleCost;
      BestPosition = Position;
      BestClock = Clock;
    }
  }

  /** set best states */
  Graph.getStateData().getElement(POSITION_STATE, Time, 0).setMean(BestPosition);
  Graph.getStateData().getElement(CLOCK_ERROR_STATE, Time, 0).setMean(BestClock);
}

void InitWithGNSS(libRSF::FactorGraph &Graph,
                  libRSF::SensorDataSet &Measurements,
                  const libRSF::FactorGraphConfig &Config,
                  libRSF::TangentPlaneConverter &LocalFrame,
                  const double TimeInitial,
                  const double WindowLength)
{
  /** find first measurement */
  double TimeFirstGNSS = 0.0;
  if(!Measurements.getTimeFirst(libRSF::DataType::Pseudorange3, TimeFirstGNSS))
  {
    PRINT_ERROR("Could not find any GNSS measurement, skip initialization!");
    return;
  }
  std::vector<libRSF::Data> PseudoRanges = Measurements.getElements(libRSF::DataType::Pseudorange3, TimeFirstGNSS);

  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.max_num_iterations = 200;
  SolverOptions.num_threads = 1;

  /** create a simple static gaussian config*/
  libRSF::FactorGraphConfig SimpleConfig = Config;
  SimpleConfig.GNSS.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
  SimpleConfig.GNSS.Type = libRSF::FactorType::Pseudorange3_ECEF;

  /** estimate SNR properties */
  double MeanSNR = 0;
  for (const libRSF::Data &PR : PseudoRanges)
  {
    MeanSNR += PR.getValue(libRSF::DataElement::SNR)(0);
  }
  MeanSNR /= static_cast<double>(PseudoRanges.size());

  /** initial estimation */
  libRSF::FactorGraph InitialGraph;
  for (const libRSF::Data &PR : PseudoRanges)
  {
    /** only use the best measurements (highest SNR) */
    if (PR.getValue(libRSF::DataElement::SNR)(0) >= MeanSNR - 0.01) // subtract 0.01 for cases, where all SNR values are zero
    {
      AddPseudorange3(InitialGraph, SimpleConfig, PR, TimeInitial);
    }
  }
  InitialGraph.solve(SolverOptions);

  /** copy position state*/
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeInitial);
  Graph.getStateData().getElement(POSITION_STATE, TimeInitial, 0).setMean(InitialGraph.getStateData().getElement(POSITION_STATE, TimeInitial, 0).getMean());

  /** copy clock error states */
  Graph.addStateWithCheck(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimeFirstGNSS);
  Graph.getStateData().getElement(CLOCK_ERROR_STATE, TimeFirstGNSS, 0).setMean(InitialGraph.getStateData().getElement(CLOCK_ERROR_STATE, TimeFirstGNSS, 0).getMean());

  /** apply RANSAC */
  //PseudoRangeRANSAC(Graph, Config, Measurements, TimeInitial, std::max(TimeFirstGNSS, TimeInitial + WindowLength), 500, 5);

  /** apply sampling */
  //PseudoRangeSampling(Graph, TimeInitial, 1e5, 100.0);

  /** copy result */
  libRSF::StateDataSet InitializationResult = Graph.getStateData();

  /** initialize ECEF<->ENU converter */
  if (Config.GNSS.Type != libRSF::FactorType::Pseudorange3_ECEF)
  {
    LocalFrame.setTangentPoint(InitializationResult.getElement(POSITION_STATE, TimeInitial).getMean());

    /** convert state in local frame */
    LocalFrame.convertStateToLocal(Graph.getStateData().getElement(POSITION_STATE, TimeInitial));

    /** convert measurements in local frame */
    LocalFrame.convertAllPseudorangesToLocal(Measurements);
  }

  /** calculate uncertainty for prior */
//  SimpleGraph.computeCovariance(POSITION_STATE, TimeInitial);
//  libRSF::StateDataSet SimpleResult = SimpleGraph.getStateData();

  /** add prior */
//  libRSF::Data PriorPoint(libRSF::DataType::Point3, TimeInitial);
//  PriorPoint.setMean(SimpleResult.getElement(POSITION_STATE, TimeInitial).getMean());
//
//  libRSF::GaussianFull<3> NoisePrior;
//  NoisePrior.setCovariance(SimpleResult.getElement(POSITION_STATE, TimeInitial).getCovarianceMatrix()*10);/**< we don't trust the initial result! */
//  Graph.addFactor<libRSF::FactorType::Prior3>(libRSF::StateID(POSITION_STATE, TimeInitial), PriorPoint, NoisePrior);
}


void InitWithUWB(libRSF::FactorGraph &Graph,
                 libRSF::SensorDataSet &Measurements,
                 const libRSF::FactorGraphConfig &Config,
                 const double TimeInitial,
                 const double WindowLength,
                 const int MinRangeNumber)
{
  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.max_num_iterations = 100;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());

  /** construct new graph */
  libRSF::FactorGraph SimpleGraph;

  /** create a simple static Gaussian config*/
  libRSF::FactorGraphConfig SimpleConfig = Config;
  SimpleConfig.Ranging.Type = libRSF::FactorType::Range3;
  SimpleConfig.Ranging.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
  SimpleConfig.Ranging.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;

  /**loop over measurements */
  double Time, TimeFirst;
  int NumberRanges = 0;
  if (Measurements.getTimeFirst(libRSF::DataType::Range3, TimeFirst))
  {
    Time = TimeFirst;
    do
    {
      /**loop over modules and add range factors*/
      for (const libRSF::Data& Range : Measurements.getElements(libRSF::DataType::Range3, Time))
      {
        AddRange3(SimpleGraph, SimpleConfig, Range, TimeInitial);
        NumberRanges++;
      }
    }
    while (Measurements.getTimeNext(libRSF::DataType::Range3, Time, Time) && ((Time - TimeInitial <= WindowLength) || (NumberRanges < MinRangeNumber)));
  }
  else
  {
    PRINT_ERROR("No UWB measurements available!");
    return;
  }

  /** solve */
  SimpleGraph.solve(SolverOptions);

  /** set initial value to graph */
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeInitial);
  Graph.getStateData().getElement(POSITION_STATE, TimeInitial, 0).setMean(SimpleGraph.getStateData().getElement(POSITION_STATE, TimeInitial).getMean());

  /** add initial uncertainty */
  if (SimpleGraph.computeCovariance(POSITION_STATE, TimeInitial))
  {
    /** get prior values */
    libRSF::Vector3 MeanPrior = SimpleGraph.getStateData().getElement(POSITION_STATE, TimeInitial).getMean();
    libRSF::Matrix33 CovPrior = SimpleGraph.getStateData().getElement(POSITION_STATE, TimeInitial).getCovarianceMatrix();

    /** Z in usually not observable*/
    MeanPrior(2) = 0;

    /** do not rely on this prior too much */
    CovPrior *= 10;

    /** create "measurement" */
    libRSF::Data PriorPoint(libRSF::DataType::Point3, TimeInitial);
    PriorPoint.setMean(MeanPrior);

    libRSF::GaussianFull<3> NoisePrior;
    NoisePrior.setCovarianceMatrix(CovPrior);

    /** add prior */
    Graph.addFactor<libRSF::FactorType::Prior3>(libRSF::StateID(POSITION_STATE, TimeInitial), PriorPoint, NoisePrior);
  }
  else
  {
    PRINT_WARNING("Could not calculate prior factor with UWB measurements, continue without!");
  }
}

void InitIMU(libRSF::FactorGraph &Graph,
             libRSF::SensorDataSet &Measurements,
             const double TimeInitial,
             const double WindowLength)
{
  /** get measurements */
  std::vector<libRSF::Data> IMU;
  IMU = Measurements.getElementsBetween(libRSF::DataType::IMU, TimeInitial, TimeInitial + WindowLength);

  if (!IMU.empty())
  {
    /** average */
    libRSF::Data IMUAverage = libRSF::AverageMeasurement(IMU);
    libRSF::Vector3 Acc = IMUAverage.getMean().head(3);
    libRSF::Vector3 TR = IMUAverage.getMean().tail(3);

    /** estimate orientation */
    libRSF::Quaternion QuatGravity;
    QuatGravity = libRSF::Quaternion::FromTwoVectors(Acc, libRSF::GRAVITY_VECTOR);

    /** estimate acceleration bias */
    Acc -= QuatGravity.conjugate() * libRSF::GRAVITY_VECTOR;

    /** add states */
    Graph.addStateWithCheck(IMU_STATE, libRSF::DataType::IMUBias, TimeInitial);
    Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeInitial);

    /** set biases */
    libRSF::Vector9 SpeedBias;
    SpeedBias << 0, 0, 0, Acc, TR;
    Graph.getStateData().getElement(IMU_STATE, TimeInitial, 0).setMean(SpeedBias);

    /** set orientation */
    libRSF::Vector4 QuatVect;
    QuatVect << QuatGravity.vec(), QuatGravity.w();
    Graph.getStateData().getElement(ORIENTATION_STATE, TimeInitial, 0).setMean(QuatVect);

    /** add prior for speed-bias */
    libRSF::Data SpeedBiasMeasurement(libRSF::DataType::IMUBias, TimeInitial);
    SpeedBiasMeasurement.setMean(SpeedBias);

    libRSF::GaussianDiagonal<9> Gauss;
    libRSF::Vector9 NoiseVect;
    NoiseVect << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    Gauss.setStdDevDiagonal(NoiseVect);
    Graph.addFactor<libRSF::FactorType::Prior9>(libRSF::StateID(IMU_STATE, TimeInitial, 0), SpeedBiasMeasurement, Gauss);

    /** add rotation prior*/
    libRSF::Data Rot(libRSF::DataType::Quaternion, TimeInitial);
    Rot.setMean(QuatVect);
    libRSF::GaussianDiagonal<3> GaussRot;
    libRSF::Vector3 NoiseRot;
    NoiseRot << libRSF::deg2rad(5), libRSF::deg2rad(5), libRSF::deg2rad(3600);
    GaussRot.setStdDevDiagonal(NoiseRot);
    Graph.addFactor<libRSF::FactorType::PriorQuat>(libRSF::StateID(ORIENTATION_STATE, TimeInitial, 0), Rot, GaussRot);
  }
  else
  {
    PRINT_WARNING("Could not initialize IMU, continue without!");
  }
}

void InitOdom(libRSF::FactorGraph &Graph,
              const libRSF::FactorType OdomType,
              const double TimeInitial)
{
  if (OdomType == libRSF::FactorType::Odom6 || OdomType == libRSF::FactorType::BetweenPose3)
  {
    /** add first orientation state */
    Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeInitial);

    /** add rotation prior prior at current state */
    libRSF::Data Rot(libRSF::DataType::Quaternion, TimeInitial);
    Rot.setMean(Graph.getStateData().getElement(ORIENTATION_STATE, TimeInitial, 0).getMean());
    libRSF::GaussianDiagonal<3> Gauss;
    libRSF::Vector3 Noise;
    Noise << libRSF::deg2rad(15), libRSF::deg2rad(15), libRSF::deg2rad(3600);
    Gauss.setStdDevDiagonal(Noise);
    Graph.addFactor<libRSF::FactorType::PriorQuat>(libRSF::StateID(ORIENTATION_STATE, TimeInitial, 0), Rot, Gauss);
  }
  if (OdomType == libRSF::FactorType::Odom4)
  {
    /** add first orientation state */
    Graph.addStateWithCheck(ANGLE_STATE, libRSF::DataType::Angle, TimeInitial);

    /** add rotation prior prior at current state */
    libRSF::Data Rot(libRSF::DataType::Angle, TimeInitial);
    Rot.setMean(libRSF::Vector1::Zero());
    libRSF::GaussianDiagonal<1> Gauss;
    Gauss.setStdDevSharedDiagonal(libRSF::deg2rad(3600));
    Graph.addFactor<libRSF::FactorType::PriorAngle>(libRSF::StateID(ANGLE_STATE, TimeInitial, 0), Rot, Gauss);
  }
}

