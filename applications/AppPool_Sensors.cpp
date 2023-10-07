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

#include "AppPool_Sensors.h"

void AddIMU(libRSF::FactorGraph &Graph,
            const libRSF::FactorGraphConfig &Config,
            const libRSF::SensorDataSet &Measurements,
            const double TimeOld,
            const double TimeNow)
{
  /** get measurements */
  double TimeOldNext;
  std::vector<libRSF::Data> MeasurementsIMU;
  if (Measurements.getTimeAbove(libRSF::DataType::IMU, TimeOld, TimeOldNext))
  {
    MeasurementsIMU = Measurements.getElementsBetween(libRSF::DataType::IMU, TimeOldNext, TimeNow);
  }

  /** do not use measurements from the future */
  if (TimeOldNext > TimeNow)
  {
    MeasurementsIMU.clear();
  }

  /** catch cases where no IMU measurements are available */
  if (MeasurementsIMU.empty())
  {
    /** find last usable measurement */
    double TimeIMU;
    if (Measurements.getTimeBelowOrEqual(libRSF::DataType::IMU, TimeNow, TimeIMU))
    {
      MeasurementsIMU = Measurements.getElements(libRSF::DataType::IMU, TimeIMU);

      /** fake a new measurement */
      MeasurementsIMU.back().setTimestamp(TimeNow);

      PRINT_WARNING("There is no IMU measurement between ", TimeOld, " and ", TimeNow, ". Instead ", TimeIMU, " is used.");
    }
    else
    {
      PRINT_WARNING("There is no IMU measurement below ", TimeNow, ". Exit IMU handling now!");
      return;
    }
  }

  if (Config.IMU.Type == libRSF::FactorType::IMUSimple)
  {
    /** reduce number of measurements */
    if (Config.IMU.Parameter(5) > 0)
    {
      MeasurementsIMU = libRSF::SampleMeasurementsDown(MeasurementsIMU, Config.IMU.Parameter(5));
    }
  }

  /** insert last 'fake' measurement as constant extrapolation */
  if (MeasurementsIMU.back().getTimestamp() != TimeNow)
  {
    MeasurementsIMU.back().setTimestamp(TimeNow);
  }

  /** construct position if missing */
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeOld);
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeNow);

  /** construct orientation states if missing */
  Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeOld);
  Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeNow);

  /** construct IMU states if missing */
  Graph.addStateWithCheck(IMU_STATE, libRSF::DataType::IMUBias, TimeOld);
  Graph.addStateWithCheck(IMU_STATE, libRSF::DataType::IMUBias, TimeNow);

  /** add factors depending on type */
  if (Config.IMU.Type == libRSF::FactorType::IMUSimple)
  {
    /** connect states multiple measurements*/
    double Time1, Time2;
    Time1 = TimeOld;
    for (const libRSF::Data &IMU : MeasurementsIMU)
    {
      Time2 = IMU.getTimestamp();

      /** add IMU variable */
      if (Time2 != TimeNow)
      {
        Graph.addState(POSITION_IMU_STATE, libRSF::DataType::Point3, Time2);
        Graph.addState(ORIENTATION_IMU_STATE, libRSF::DataType::Quaternion, Time2);
        Graph.addState(IMU_STATE, libRSF::DataType::IMUBias, Time2);
      }

      /** construct noise model */
      const double DeltaTime = Time2 - Time1;
      const double StdDevAcc = Config.IMU.Parameter(0) * sqrt(1 / DeltaTime);
      const double StdDevTR = Config.IMU.Parameter(1) * sqrt(1 / DeltaTime);
      const double StdDevBiasAcc = Config.IMU.Parameter(2) * sqrt(DeltaTime);
      const double StdDevBiasTR = Config.IMU.Parameter(3) * sqrt(DeltaTime);
      const double StdDevDynamic = Config.IMU.Parameter(4);

      libRSF::Vector15 NoiseVect;
      NoiseVect << StdDevAcc, StdDevAcc, StdDevAcc,
          StdDevDynamic, StdDevDynamic, StdDevDynamic,
          StdDevTR, StdDevTR, StdDevTR,
          StdDevBiasAcc, StdDevBiasAcc, StdDevBiasAcc,
          StdDevBiasTR, StdDevBiasTR, StdDevBiasTR;

      libRSF::GaussianDiagonal<15> IMUNoise;
      IMUNoise.setStdDevDiagonal(NoiseVect);

      /** set factor IDs */
      libRSF::StateID PosID1, PosID2, RotID1, RotID2;
      if (Time1 == TimeOld && Time2 != TimeNow) /**< first measurement */
      {
        PosID1 = libRSF::StateID(POSITION_STATE, Time1);
        PosID2 = libRSF::StateID(POSITION_IMU_STATE, Time2);
        RotID1 = libRSF::StateID(ORIENTATION_STATE, Time1);
        RotID2 = libRSF::StateID(ORIENTATION_IMU_STATE, Time2);
      }
      else if (Time1 != TimeOld && Time2 == TimeNow) /**< last (fake) measurement */
      {
        PosID1 = libRSF::StateID(POSITION_IMU_STATE, Time1);
        PosID2 = libRSF::StateID(POSITION_STATE, Time2);
        RotID1 = libRSF::StateID(ORIENTATION_IMU_STATE, Time1);
        RotID2 = libRSF::StateID(ORIENTATION_STATE, Time2);
      }
      else if (Time1 == TimeOld && Time2 == TimeNow) /**< single measurement (special case) */
      {
        PosID1 = libRSF::StateID(POSITION_STATE, Time1);
        PosID2 = libRSF::StateID(POSITION_STATE, Time2);
        RotID1 = libRSF::StateID(ORIENTATION_STATE, Time1);
        RotID2 = libRSF::StateID(ORIENTATION_STATE, Time2);
      }
      else /**< measurements between */
      {
        PosID1 = libRSF::StateID(POSITION_IMU_STATE, Time1);
        PosID2 = libRSF::StateID(POSITION_IMU_STATE, Time2);
        RotID1 = libRSF::StateID(ORIENTATION_IMU_STATE, Time1);
        RotID2 = libRSF::StateID(ORIENTATION_IMU_STATE, Time2);
      }
      Graph.addFactor<libRSF::FactorType::IMUSimple>(PosID1, RotID1, libRSF::StateID(IMU_STATE, Time1), PosID2, RotID2, libRSF::StateID(IMU_STATE, Time2), IMU, IMUNoise);
      Time1 = Time2;
    }
  }
  else if (Config.IMU.Type == libRSF::FactorType::IMUPretintegration)
  {
    /** create pre-integration */
    const libRSF::Vector6 Bias = Graph.getStateData().getElement(IMU_STATE, TimeOld).getMean().tail(6);
    libRSF::IMUPreintegrator PreInt(Bias.head(3), Bias.tail(3), Config.IMU.Parameter(0), Config.IMU.Parameter(1), Config.IMU.Parameter(2), Config.IMU.Parameter(3), TimeOld);
    /** pre-integrate measurements */
    for (const libRSF::Data &IMU : MeasurementsIMU)
    {
      PreInt.addMeasurement(IMU);
    }

    /** add factor */
    libRSF::StateList IMUStates;
    IMUStates.add(POSITION_STATE, TimeOld);
    IMUStates.add(ORIENTATION_STATE, TimeOld);
    IMUStates.add(IMU_STATE, TimeOld);
    IMUStates.add(POSITION_STATE, TimeNow);
    IMUStates.add(ORIENTATION_STATE, TimeNow);
    IMUStates.add(IMU_STATE, TimeNow);
    Graph.addIMUPreintegrationFactor(IMUStates, PreInt.getPreintegratedState());
  }
}

void AddClockModel(libRSF::FactorGraph &Graph,
                   const libRSF::FactorGraphConfig &Config,
                   const std::string &ClockErrorNameStart,
                   const std::string &ClockDriftNameStart,
                   const std::string &ClockErrorNameEnd,
                   const std::string &ClockDriftNameEnd,
                   const double TimeNow)
{
  /** construct clock error if missing */
  Graph.addStateWithCheck(ClockErrorNameEnd, libRSF::DataType::ClockError, TimeNow);

  /** get previous timestamp */
  double TimeOld = 0.0;
  if (!Graph.getStateData().getTimePrev(ClockErrorNameStart, TimeNow, TimeOld))
  {
    PRINT_ERROR("No previous state available!");
  }

  switch (Config.ClockModel.Type)
  {
    case libRSF::FactorType::ConstDrift1:
    {
      /** add drift states if missing */
      Graph.addStateWithCheck(ClockDriftNameStart, libRSF::DataType::ClockDrift, TimeOld);
      Graph.addStateWithCheck(ClockDriftNameEnd, libRSF::DataType::ClockDrift, TimeNow);

      libRSF::GaussianDiagonal<2> NoiseCCED;
      NoiseCCED.setStdDevDiagonal(Config.ClockModel.Parameter);

      Graph.addFactor<libRSF::FactorType::ConstDrift1>(
          libRSF::StateID(ClockErrorNameStart, TimeOld),
          libRSF::StateID(ClockDriftNameStart, TimeOld),
          libRSF::StateID(ClockErrorNameEnd, TimeNow),
          libRSF::StateID(ClockDriftNameEnd, TimeNow),
          NoiseCCED);
    }
    break;

    case libRSF::FactorType::ConstVal1:
    {
      libRSF::GaussianDiagonal<1> NoiseCCE;
      NoiseCCE.setStdDevDiagonal(Config.ClockModel.Parameter);

      Graph.addFactor<libRSF::FactorType::ConstVal1>(
          libRSF::StateID(ClockErrorNameStart, TimeOld),
          libRSF::StateID(ClockErrorNameEnd, TimeNow),
          NoiseCCE);
    }
    break;

    default:
      PRINT_ERROR("Wrong clock model type!");
      break;
  }
}

void AddOdometry(libRSF::FactorGraph &Graph,
                 const libRSF::FactorType FactorConfig,
                 const libRSF::DataType SensorConfig,
                 const libRSF::SensorDataSet &Measurements,
                 const double TimeOld,
                 const double TimeNow)
{
  /** get measurements */
  double TimeOldNext;
  std::vector<libRSF::Data> MeasurementsOdom;
  if (Measurements.getTimeAbove(SensorConfig, TimeOld, TimeOldNext))
  {
    MeasurementsOdom = Measurements.getElementsBetween(SensorConfig, TimeOldNext, TimeNow);
  }

  /** do not use measurements from the future */
  if (TimeOldNext > TimeNow)
  {
    MeasurementsOdom.clear();
  }

  /** catch cases where no Odom measurements are available */
  if (MeasurementsOdom.empty())
  {
    /** find last usable measurement */
    double TimeOdom;
    if (Measurements.getTimeBelowOrEqual(SensorConfig, TimeNow, TimeOdom))
    {
      /** use previous measurement */
      libRSF::Data Odom;
      Measurements.getElement(SensorConfig, TimeOdom, 0, Odom);
      MeasurementsOdom.emplace_back(Odom);

      PRINT_WARNING("There is no Odometry measurement between ", TimeOld, " and ", TimeNow, ". Instead ", TimeOdom, " is used.");
    }
    else
    {
      /** create fake zero measurement */
      libRSF::Data OdomZero(libRSF::DataType::Odom3, TimeNow);
      const libRSF::Vector6 MeanZero = libRSF::Vector6::Zero();
      const libRSF::Vector6 CovZero = libRSF::Vector6::Ones() * 1e-4;
      OdomZero.setMean(MeanZero);
      OdomZero.setCovarianceDiagonal(CovZero);
      MeasurementsOdom.emplace_back(OdomZero);

      PRINT_WARNING("There is no Odometry measurement below ", TimeNow, ". Using a faked zero-measurement!");
    }
  }

  /** construct position if missing */
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeOld);
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeNow);

  switch (FactorConfig)
  {
    case libRSF::FactorType::BetweenPose3:
    {
      /** integrate odometry to relative pose */
      libRSF::OdometryIntegrator Integrator;
      double T1 = TimeOld;
      double T2 = 0.0;
      for (const libRSF::Data &Odom : MeasurementsOdom)
      {
        T2 = Odom.getTimestamp();
        Integrator.addMeasurement(Odom, T2 - T1);
        T1 = T2;
      }
      /** add last measurement again to cover the full period between TimeOld and TimeNow */
      Integrator.addMeasurement(MeasurementsOdom.back(), TimeNow - T2);

      /** convert to relative measurement */
      libRSF::Data PoseInt(libRSF::DataType::Pose3, TimeNow);
      PoseInt.setMean(Integrator.getJointPose());

      libRSF::GaussianFull<6> NoiseInt;
      NoiseInt.setCovarianceMatrix(Integrator.getJointCovOnManifold());

      /** construct orientation states if missing */
      Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeOld);
      Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeNow);

      /** add factor */
      Graph.addFactor<libRSF::FactorType::BetweenPose3>(libRSF::StateID(POSITION_STATE, TimeOld),
                                                        libRSF::StateID(ORIENTATION_STATE, TimeOld),
                                                        libRSF::StateID(POSITION_STATE, TimeNow),
                                                        libRSF::StateID(ORIENTATION_STATE, TimeNow),
                                                        PoseInt,
                                                        NoiseInt);
    }
    break;

    case libRSF::FactorType::Odom4:
    {
      /** average if required */
      const libRSF::Data Odom = libRSF::AverageMeasurement(MeasurementsOdom);

      /** construct orientation states if missing */
      Graph.addStateWithCheck(ANGLE_STATE, libRSF::DataType::Angle, TimeOld);
      Graph.addStateWithCheck(ANGLE_STATE, libRSF::DataType::Angle, TimeNow);

      /** construct error model */
      libRSF::Vector4 Cov;
      Cov << Odom.getCovarianceDiagonal().head(3), Odom.getCovarianceDiagonal().tail(1);
      libRSF::GaussianDiagonal<4> NoiseOdom;
      NoiseOdom.setCovarianceDiagonal(Cov);

      /** add factor */
      Graph.addFactor<libRSF::FactorType::Odom4>(libRSF::StateID(POSITION_STATE, TimeOld),
                                                 libRSF::StateID(ANGLE_STATE, TimeOld),
                                                 libRSF::StateID(POSITION_STATE, TimeNow),
                                                 libRSF::StateID(ANGLE_STATE, TimeNow),
                                                 Odom,
                                                 NoiseOdom);
    }
    break;

    case libRSF::FactorType::Odom4_ECEF:
    {
      /** average if required */
      const libRSF::Data Odom = libRSF::AverageMeasurement(MeasurementsOdom);

      /** construct orientation states if missing */
      Graph.addStateWithCheck(ANGLE_STATE, libRSF::DataType::Angle, TimeOld);
      Graph.addStateWithCheck(ANGLE_STATE, libRSF::DataType::Angle, TimeNow);

      /** construct error model */
      libRSF::Vector4 Cov;
      Cov << Odom.getCovarianceDiagonal().head(3), Odom.getCovarianceDiagonal().tail(1);
      libRSF::GaussianDiagonal<4> NoiseOdom;
      NoiseOdom.setCovarianceDiagonal(Cov);

      /** add factor */
      Graph.addFactor<libRSF::FactorType::Odom4_ECEF>(libRSF::StateID(POSITION_STATE, TimeOld),
                                                      libRSF::StateID(ANGLE_STATE, TimeOld),
                                                      libRSF::StateID(POSITION_STATE, TimeNow),
                                                      libRSF::StateID(ANGLE_STATE, TimeNow),
                                                      Odom,
                                                      NoiseOdom);
    }
    break;

    case libRSF::FactorType::Odom6:
    {
      /** average if required */
      const libRSF::Data Odom = libRSF::AverageMeasurement(MeasurementsOdom);

      /** construct orientation states if missing */
      Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeOld);
      Graph.addStateWithCheck(ORIENTATION_STATE, libRSF::DataType::Quaternion, TimeNow);

      /** construct error model */
      libRSF::GaussianDiagonal<6> NoiseOdom;
      NoiseOdom.setCovarianceDiagonal(Odom.getCovarianceDiagonal());

      /** add factor */
      Graph.addFactor<libRSF::FactorType::Odom6>(libRSF::StateID(POSITION_STATE, TimeOld),
                                                 libRSF::StateID(ORIENTATION_STATE, TimeOld),
                                                 libRSF::StateID(POSITION_STATE, TimeNow),
                                                 libRSF::StateID(ORIENTATION_STATE, TimeNow),
                                                 Odom,
                                                 NoiseOdom);
    }

    break;

    default:
      PRINT_ERROR("Wrong odometry type!");
      break;
  }
}

void AddMotionModel(libRSF::FactorGraph &Graph,
                    const libRSF::FactorGraphConfig &Config,
                    const double TimeOld,
                    const double TimeNow)
{
  // TODO...
}

void AddPressure(libRSF::FactorGraph &Graph,
                 const libRSF::FactorType FactorConfig,
                 const libRSF::SensorDataSet &Measurements,
                 const double TimeOld,
                 const double TimeNow)
{
  /** remember if we had an initial value */
  static bool IsInitialized = false;

  /** store the last measurement */
  static libRSF::Data Pressure(libRSF::DataType::AirPressure, TimeNow);

  /** get measurements */
  double TimeOldNext;
  std::vector<libRSF::Data> MeasurementsPressure;
  if (Measurements.getTimeAbove(libRSF::DataType::AirPressure, TimeOld, TimeOldNext))
  {
    MeasurementsPressure = Measurements.getElementsBetween(libRSF::DataType::AirPressure, TimeOldNext, TimeNow);
  }

  /** do not use measurements from the future */
  if (TimeOldNext > TimeNow)
  {
    MeasurementsPressure.clear();
  }

  /** average pressure over past interval */
  libRSF::Data PressureNew(libRSF::DataType::AirPressure, TimeNow);
  if (!MeasurementsPressure.empty())
  {
    PressureNew = libRSF::AverageMeasurement(MeasurementsPressure);

    /** assume constant pressure for the first iteration */
    if (!IsInitialized)
    {
      Pressure = PressureNew;
      IsInitialized = true;
    }
  }
  else
  {
    PRINT_WARNING("There is no barometric pressure measurement between ", TimeOld, " and ", TimeNow, ". Assume constant pressure!");
    if (!IsInitialized)
    {
      PRINT_ERROR("Without initialization I could not assume a constant pressure!");
      return;
    }
  }

  /** calculate pressure difference */
  libRSF::Data PressureDiff(libRSF::DataType::AirPressureDiff, TimeNow);
  PressureDiff.setMean(PressureNew.getMean() - Pressure.getMean());
  PressureDiff.setCovarianceDiagonal(PressureNew.getCovarianceDiagonal() + Pressure.getCovarianceDiagonal());

  /** create noise model */
  libRSF::GaussianDiagonal<1> Noise;
  Noise.setCovarianceDiagonal(PressureDiff.getCovarianceDiagonal());

  /** construct position if missing */
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeOld);
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimeNow);

  /** add pressure difference factor */
  switch (FactorConfig)
  {
    case libRSF::FactorType::PressureDiff3:
      Graph.addFactor<libRSF::FactorType::PressureDiff3>(libRSF::StateID(POSITION_STATE, TimeOld),
                                                         libRSF::StateID(POSITION_STATE, TimeNow),
                                                         PressureDiff,
                                                         Noise);
      break;

    default:
      PRINT_ERROR("Wrong clock model type!");
      break;
  }

  /** store pressure for the next iteration */
  Pressure = PressureNew;
}

void AddRange3(libRSF::FactorGraph &Graph,
               const libRSF::FactorGraphConfig &Config,
               const libRSF::Data &Range,
               const double TimePosition)
{
  AddRangeGeneric<libRSF::FactorType::Range3>(Graph, Config, Range, TimePosition);
}

void AddRange2(libRSF::FactorGraph &Graph,
               const libRSF::FactorGraphConfig &Config,
               const libRSF::Data &Range,
               const double TimePosition)
{
  AddRangeGeneric<libRSF::FactorType::Range2>(Graph, Config, Range, TimePosition);
}

void AddRangeToPoint2(libRSF::FactorGraph &Graph,
                      const libRSF::FactorGraphConfig &Config,
                      const libRSF::Data &Range,
                      const double TimePosition)
{
  AddRangeGeneric<libRSF::FactorType::RangeToPoint2>(Graph, Config, Range, TimePosition);
}

void AddUWB(libRSF::FactorGraph &Graph,
            const libRSF::FactorGraphConfig &Config,
            const libRSF::SensorDataSet &Measurements,
            const double TimeOld,
            const double TimeNow)
{
  /** get measurements */
  double TimeOldNext;
  std::vector<libRSF::Data> MeasurementsUWB;
  if (Measurements.getTimeAbove(libRSF::DataType::Range3, TimeOld, TimeOldNext))
  {
    MeasurementsUWB = Measurements.getElementsBetween(libRSF::DataType::Range3, TimeOldNext, TimeNow);
  }

  if (!MeasurementsUWB.empty())
  {
    /** loop over measurements */
    double TimeState;
    double TimeUWBOld = MeasurementsUWB.at(0).getTimestamp() - 1;
    for (const libRSF::Data &Range : MeasurementsUWB)
    {
      const double TimeUWB = Range.getTimestamp();

      /** if time changes */
      if (TimeUWB != TimeUWBOld)
      {
        /** decide to which state it should be connected */
        if (!Graph.getStateData().getTimeCloseTo(POSITION_STATE, TimeUWB, TimeState))
        {
          PRINT_ERROR("There is no exiting Position!");
          return;
        }
        TimeUWBOld = TimeUWB;
      }

      /** add range factor */
      AddRange3(Graph, Config, Range, TimeState);
    }
  }
  else
  {
    PRINT_WARNING("Found no UWB measurement!");
  }
}

void AddGNSS(libRSF::FactorGraph &Graph,
             const libRSF::FactorGraphConfig &Config,
             const libRSF::SensorDataSet &Measurements,
             const double TimeOld,
             const double TimeNow)
{
  /** get measurements */
  double TimeOldNext = NAN;
  std::vector<libRSF::Data> MeasurementsGNSS;
  if (Measurements.getTimeAbove(libRSF::DataType::Pseudorange3, TimeOld, TimeOldNext))
  {
    MeasurementsGNSS = Measurements.getElementsBetween(libRSF::DataType::Pseudorange3, TimeOldNext, TimeNow);
  }

  if (!MeasurementsGNSS.empty())
  {
    /** loop over measurements */
    double TimeState = NAN;
    double TimeGNSSOld = MeasurementsGNSS.at(0).getTimestamp() - 1;
    for (const libRSF::Data &Pseudorange : MeasurementsGNSS)
    {
      const double TimeGNSS = Pseudorange.getTimestamp();

      /** check if clock error already exists*/
      if (!Graph.getStateData().checkElement(CLOCK_ERROR_STATE, TimeGNSS))
      {
        if (Config.ClockModel.IsActive)
        {
          if (Graph.getStateData().countElements(CLOCK_ERROR_STATE) > 0)
          {
            AddClockModel(Graph, Config, CLOCK_ERROR_STATE, CLOCK_DRIFT_STATE, CLOCK_ERROR_STATE, CLOCK_DRIFT_STATE, TimeGNSS);
          }
        }
        else
        {
          /** add state in case the clock model is deactivated */
          Graph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimeGNSS);
        }
      }

      if (Config.GNSS.Type == libRSF::FactorType::Pseudorange3_Bias)
      {
        /** different bias for each satellite system*/
        const std::string InterSysBiasName = addSatSysToName(SYSTEM_BIAS_STATE, static_cast<int>(Pseudorange.getValue(libRSF::DataElement::SatSys)(0)));

        /** check if inter-system bias exists*/
        if (Graph.getStateData().countElements(InterSysBiasName) < 1)
        {
          /** create a single ISB state so far in the future, that it will never be marginalized */
          AddInterSystemBiasModel(Graph, Config, InterSysBiasName, TimeGNSS + 1e20);
        }
      }

      /** if time changes, find next state */
      if (TimeGNSS != TimeGNSSOld)
      {
        if (!Graph.getStateData().getTimeCloseTo(POSITION_STATE, TimeGNSS, TimeState))
        {
          PRINT_ERROR("There is no exiting Position!");
          return;
        }
        TimeGNSSOld = TimeGNSS;
      }

      /** add pseudorange factor */
      AddPseudorange3(Graph, Config, Pseudorange, TimeState);
    }
  }
  else
  {
    PRINT_WARNING("Found no GNSS measurement!");
  }
}

void AddInterSystemBiasModel(libRSF::FactorGraph &Graph,
                             const libRSF::FactorGraphConfig &Config,
                             const std::string &ISBName,
                             const double TimeBias)
{
  /** add new inter system bias state */
  Graph.addState(ISBName, libRSF::DataType::Value1, TimeBias);

  if (ISBName == SYSTEM_BIAS_STATE)
  {
    /** set to zero constant, because GPS is the unbiased reference */
    Graph.getStateData().getElement(ISBName, TimeBias).setMean(libRSF::Vector1::Zero());
    Graph.setConstant(ISBName, TimeBias);
  }
  else
  {
    /** add zero-prior */
    libRSF::GaussianDiagonal<1> NoisePrior;
    NoisePrior.setStdDevSharedDiagonal(Config.GNSS.Parameter(0));

    libRSF::Data Zero(libRSF::DataType::Value1, TimeBias);
    Zero.setMean(libRSF::Vector1::Zero());

    Graph.addFactor<libRSF::FactorType::Prior1>(libRSF::StateID(ISBName, TimeBias),
                                                Zero,
                                                NoisePrior);
  }
}

void AddPseudorange3(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     const libRSF::Data &Pseudorange,
                     const double TimePosition)
{
  if (Config.GNSS.Type == libRSF::FactorType::Pseudorange3)
  {
    AddPseudorange3Generic<libRSF::FactorType::Pseudorange3>(Graph, Config, Pseudorange, TimePosition);
  }
  else if (Config.GNSS.Type == libRSF::FactorType::Pseudorange3_Bias)
  {
    AddPseudorange3Generic<libRSF::FactorType::Pseudorange3_Bias>(Graph, Config, Pseudorange, TimePosition);
  }
  else if (Config.GNSS.Type == libRSF::FactorType::Pseudorange3_ECEF)
  {
    AddPseudorange3Generic<libRSF::FactorType::Pseudorange3_ECEF>(Graph, Config, Pseudorange, TimePosition);
  }
  else
  {
    PRINT_ERROR("Wrong pseudo range factor type!");
  }
}

bool AddLoopClosures(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     const libRSF::SensorDataSet &Measurements,
                     const double TimeOld,
                     const double TimeNow)
{
  bool HasLoop = false;
  if (Measurements.checkID(libRSF::DataType::LoopClosure))
  {
    /** get measurements */
    double TimeOldNext = NAN;
    std::vector<libRSF::Data> Loops;
    if (Measurements.getTimeAbove(libRSF::DataType::LoopClosure, TimeOld, TimeOldNext))
    {
      /** do not use measurements from the future */
      if (TimeOldNext <= TimeNow)
      {
        Loops = Measurements.getElementsBetween(libRSF::DataType::LoopClosure, TimeOldNext, TimeNow);
      }
    }

    /** add loop closure factors */
    for (const libRSF::Data &Loop : Loops)
    {
      if (Loop.getValue(libRSF::DataElement::Similarity)(0) >= Config.LoopClosure.Parameter(0))
      {
        switch (Config.LoopClosure.Type)
        {
          case libRSF::FactorType::ConstVal3:
            AddLoopClosureGeneric<libRSF::FactorType::ConstVal3, 3>(Graph, Config, Loop);
            break;

          case libRSF::FactorType::Loop2:
            AddLoopClosureGeneric<libRSF::FactorType::Loop2, 2>(Graph, Config, Loop);
            break;

          case libRSF::FactorType::LoopPose2:
            AddLoopClosureGeneric<libRSF::FactorType::LoopPose2, 3>(Graph, Config, Loop);
            break;

          default:
            PRINT_WARNING("No handler for factor type: ", Config.LoopClosure.Type);
            break;
        }
        HasLoop = true;
      }
    }
  }
  return HasLoop;
}

std::string addSatSysToName(const std::string &BaseName, const int SatSys)
{
  std::string Name;
  if (SatSys != 1)
  {
    Name = std::string(BaseName) + "_" + std::to_string(SatSys);
  }
  else
  {
    /** GPS uses standard name*/
    Name = BaseName;
  }
  return Name;
}
