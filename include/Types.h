/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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
* @file Types.h
* @author Tim Pfeifer
* @date 10.04.2019
* @brief Collection of types and enums.
* @copyright GNU Public License.
*
*/

#ifndef TYPES_H
#define TYPES_H

#include <map>
#include <iostream>

namespace libRSF
{
  /** config types */
  enum class ErrorModelType {Gaussian, DCE, cDCE, SC, DCS, GMM};

  enum class ErrorModelMixtureType {None, MaxMix, SumMix, SumMixSpecial, MaxSumMix};
  enum class ErrorModelTuningType {None, EM, EM_MAP, VBI};

  enum class SolutionType {None, Batch, Smoother, Window, Filter};

  /** types of used factors (Remember to add new types to FactorTypeDict below!) */
  enum class FactorType
  {
    ConstVal1, ConstVal2, ConstVal3,
    ConstDrift1, ConstDrift2, ConstDrift3,
    BetweenValue1, BetweenValue2, BetweenValue3,
    BetweenPose2, BetweenPose3,
    BetweenQuaternion,
    Point2Reg,
    Point2RegPose,
    Range2, Range3, Pseudorange2, Pseudorange3, Pseudorange3_ECEF,
    Odom2, Odom2Diff, Odom4, Odom4_ECEF, Odom6,
    Prior1, Prior2, Prior3, Prior4, Prior9, PriorQuat, PriorAngle,
    IMUSimple, IMUPretintegration,
    Repelling,
    TrackingDetection,
    TrackingDetectionDim, TrackingDetectionRot, TrackingDetectionVel,
    TrackingDetectionDimRot, TrackingDetectionVelDim, TrackingDetectionVelRot,
    TrackingDetectionVelDimRot,
    Marginal
  };

  /** more general classification */
  enum class AbstractFactorType
  {
    Ranging, GNSS, ClockModel, Odometry, IMU, Radar, Laser, Vision, MotionModel, LoopClosure, Prior
  };

  /** types of state data */
  enum class StateType {Point1, Point2, Point3,
                        PointID2, PointID3,
                        Pose2, Pose3,
                        Quat, Angle, UnitCircle,
                        GMM, Switch,
                        Covariance1,
                        Error1, Error2, Error3,
                        Cost, CostGradient,
                        ClockError, ClockDrift,
                        IMUBias,
                        IterationSummary};
  enum class StateElement {Timestamp,
                           Mean, Covariance,
                           Cost, Gradient, Hessian,
                           Weight,
                           TF,
                           Other,
                           ID, Conf, Idx, WLH, R, R_Quat, Class, Key, Velocity,
                           DurationSolver, DurationMarginal, DurationAdaptive, DurationTotal,
                           IterationSolver, IterationAdaptive};

  enum class SensorType {Range2, Range3,
                         Pseudorange3, Pseudorange2,
                         Odom2, Odom3, Odom2Diff,
                         Odom3Radar, Odom3Laser, Odom3VIO,
                         Point1, Point2, Point3,
                         GT1, GT2, GT3,
                         Point2Radar,
                         Point1Set, Point2Set, Point3Set,
                         Point2SetRadar,
                         PointID2, PointID3,
                         Angle, Quaternion,
                         Pose2, Pose3,
                         LoopClosure,
                         IMU,
                         Val1,
                         Other};
  enum class SensorElement {Timestamp, TimestampRef,
                            Mean, StdDev, Covariance,
                            SatPos, SatElevation, SNR, SatID,
                            WheelBase, TF,
                            ID, Conf, Idx, Velocity, WLH, R, R_Quat, Class, Key};

  /** cout enums */
  std::ostream& operator << (std::ostream& Os, const SensorType& Type);
  std::ostream& operator << (std::ostream& Os, const StateType& Type);
  std::ostream& operator << (std::ostream& Os, const FactorType& Type);
  std::ostream& operator << (std::ostream& Os, const SolutionType& Type);
  std::ostream& operator << (std::ostream& Os, const ErrorModelType& Type);

  /** dictionary to translate factor types into related sensor types */
  const std::map<FactorType, SensorType> FactorSensorDict =
  {
    {FactorType::Range2, SensorType::Range2},
    {FactorType::Range3, SensorType::Range3},
    {FactorType::BetweenPose2, SensorType::Pose2},
    {FactorType::BetweenPose3, SensorType::Pose3},
    {FactorType::Point2Reg, SensorType::Point2Set},
    {FactorType::Pseudorange2, SensorType::Pseudorange2},
    {FactorType::Pseudorange3, SensorType::Pseudorange3},
    {FactorType::Pseudorange3_ECEF, SensorType::Pseudorange3},
    {FactorType::IMUPretintegration, SensorType::IMU},
    {FactorType::IMUSimple, SensorType::IMU},
    {FactorType::Odom2, SensorType::Odom2},
    {FactorType::Odom2Diff, SensorType::Odom2Diff},
    {FactorType::Odom4_ECEF, SensorType::Odom3},
    {FactorType::Odom4, SensorType::Odom3},
    {FactorType::Odom6, SensorType::Odom3}
  };

  /** dictionaries translate strings (from files) to enums*/
  const std::map<std::string, FactorType> FactorTypeDict =
  {
    {"const_val1",FactorType::ConstVal1},
    {"const_val2",FactorType::ConstVal2},
    {"const_val3",FactorType::ConstVal3},
    {"const_drift1",FactorType::ConstDrift1},
    {"const_drift2",FactorType::ConstDrift2},
    {"const_drift3",FactorType::ConstDrift3},
    {"between_val1",FactorType::BetweenValue1},
    {"between_val2",FactorType::BetweenValue2},
    {"between_val3",FactorType::BetweenValue3},
    {"between_pose2",FactorType::BetweenPose2},
    {"between_pose3",FactorType::BetweenPose3},
    {"range2",FactorType::Range2},
    {"range3",FactorType::Range3},
    {"pseudorange2",FactorType::Pseudorange2},
    {"pseudorange3",FactorType::Pseudorange3},
    {"pseudorange3_ecef",FactorType::Pseudorange3_ECEF},
    {"odom2",FactorType::Odom2},
    {"odom2diff",FactorType::Odom2Diff},
    {"odom4_ecef",FactorType::Odom4_ECEF},
    {"odom4",FactorType::Odom4},
    {"odom6",FactorType::Odom6},
    {"prior1",FactorType::Prior1},
    {"prior2",FactorType::Prior2},
    {"prior3",FactorType::Prior3},
    {"prior4",FactorType::Prior4},
    {"prior9",FactorType::Prior9},
    {"prior_quat",FactorType::PriorQuat},
    {"prior_angle",FactorType::PriorAngle},
    {"imu_pre",FactorType::IMUPretintegration},
    {"imu_simple",FactorType::IMUSimple},
    {"repelling",FactorType::Repelling},
    {"tracking_detection",FactorType::TrackingDetection},
    {"marginal",FactorType::Marginal},
    {"point2_reg",FactorType::Point2Reg}
  };

  const std::map<std::string, AbstractFactorType> AbstractFactorTypeDict =
  {
    {"clock",AbstractFactorType::ClockModel},
    {"gnss",AbstractFactorType::GNSS},
    {"imu",AbstractFactorType::IMU},
    {"laser",AbstractFactorType::Laser},
    {"odom",AbstractFactorType::Odometry},
    {"radar",AbstractFactorType::Radar},
    {"range",AbstractFactorType::Ranging},
    {"vision",AbstractFactorType::Vision},
    {"motion",AbstractFactorType::MotionModel},
    {"loop",AbstractFactorType::LoopClosure},
    {"prior",AbstractFactorType::Prior}
  };

  const std::map<std::string, ErrorModelType> ErrorModelTypeDict =
  {
    {"gauss",ErrorModelType::Gaussian},
    {"sc",ErrorModelType::SC},
    {"dcs",ErrorModelType::DCS},
    {"dce",ErrorModelType::DCE},
    {"cdce",ErrorModelType::cDCE},
    {"gmm",ErrorModelType::GMM}
  };

  const std::map<std::string, ErrorModelMixtureType> ErrorModelMixtureTypeDict =
  {
    {"mm",ErrorModelMixtureType::MaxMix},
    {"sm",ErrorModelMixtureType::SumMix},
    {"sm_special",ErrorModelMixtureType::SumMixSpecial},
    {"msm",ErrorModelMixtureType::MaxSumMix}
  };

  const std::map<std::string, ErrorModelTuningType> ErrorModelTuningTypeDict =
  {
    {"em",ErrorModelTuningType::EM},
    {"em_map",ErrorModelTuningType::EM_MAP},
    {"vbi",ErrorModelTuningType::VBI},
    {"none",ErrorModelTuningType::None}
  };

  const std::map<std::string, SolutionType> SolutionTypeDict =
  {
    {"batch",SolutionType::Batch},
    {"smoother",SolutionType::Smoother},
    {"window",SolutionType::Window},
    {"filter",SolutionType::Filter},
    {"none",SolutionType::None}
  };

  template <typename DictType>
  bool TranslateSafe(const DictType Dict, typename DictType::key_type const &Original, typename DictType::mapped_type &Translation)
  {
    try
    {
      Translation = Dict.at(Original);
    }
    catch (std::out_of_range& e)
    {
      /** Due to dependency cycle, we can not use PRINT_ERROR() here! */
      std::cerr << "Error in Types.h | Line 217 | TranslateSafe(): Dictionary entry is missing: " << Original;
      return false;
    }
    return true;
  }
}

#endif // TYPES_H
