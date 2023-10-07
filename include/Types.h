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
* @file Types.h
* @author Tim Pfeifer
* @date 10.04.2019
* @brief Collection of types and enums.
* @copyright GNU Public License.
*
*/

#ifndef TYPES_H
#define TYPES_H

#include "DataConfig.h"

#include <map>
#include <iostream>

namespace libRSF
{
  /** config types */
  enum class ErrorModelType {Gaussian, DCE, cDCE, SC, DCS, GMM};

  enum class ErrorModelMixtureType {None, MaxMix, SumMix, SumMixSpecial, MaxSumMix};
  enum class ErrorModelTuningType {None, EM, EM_MAP, VBI, VBI_Full};

  enum class SolutionType {None, Batch, Smoother, SmootherRT, Window, Filter};

  /** types of used factors (Remember to add new types to FactorTypeDict below!) */
  enum class FactorType
  {
    ConstVal1, ConstVal2, ConstVal3,
    ConstDrift1, ConstDrift2, ConstDrift3,
    ConstQuaternion,
    BetweenValue1, BetweenValue2, BetweenValue3,
    Loop1, Loop2, Loop3,
    LoopPose2,
    BetweenPose2, BetweenPose3,
    BetweenQuaternion,
    BetweenBearingRange2,
    Point2Reg,
    Point2RegPose,
    Range2, Range3, Pseudorange2, Pseudorange3, Pseudorange3_ECEF, Pseudorange3_Bias,
    RangeToPoint2,RangeToPoint3,
    Odom2, Odom2Diff, Odom4, Odom4_ECEF, Odom6,
    Prior1, Prior2, Prior3, Prior4, Prior9, PriorQuat, PriorAngle,
    IMUSimple, IMUPretintegration,
    PressureDiff2, PressureDiff3,
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
    Ranging, GNSS, ClockModel, Odometry, IMU, Radar, Laser, Vision, MotionModel, LoopClosure, Prior, Pressure
  };

  /** types of data that have to be stored */
  enum class DataType
  {
    Point1, Point2, Point3,                               /**< translation */
    Quaternion, Angle, UnitCircle,                        /**< rotation */
    Pose2, Pose3,                                         /**< translation + rotation*/
    PoseBetween2, PoseBetween3,                           /**< translation + rotation between two timestamps*/
    BearingRangeID2,                                      /**< BR to a specific ID*/
    PointID2,                                             /**< Point with a specific ID */
    PointConfID2,                                         /**< Point with ID and confidence */
    BoundingBox3,                                         /**< bounding boxes from object detection */
    ClockError, ClockDrift,                               /**< GNSS receiver clock error */
    IMUBias,                                              /**< IMU Speedbias [Vel, B_Acc, B_Gyr] */
    GMM1, GMM2, Switch, Covariance1, Covariance2,         /**< dynamic error models */
    Range2, Range3,                                       /**< range to fixed point */
    RangeLM2, RangeLM3,                                   /**< range to landmark with unknown position*/
    Pseudorange3, Pseudorange2,                           /**< GNSS pseudo-range */
    Odom2, Odom3, Odom2Diff,                              /**< wheel based odometry */
    Odom3Radar, Odom3Laser, Odom3VIO,                     /**< odometry based on other sensors */
    Point2Radar,                                          /**< point with doppler velocity*/
    Point1Set, Point2Set, Point3Set,                      /**< two corresponding points */
    Point2SetRadar,                                       /**< two corresponding points with doppler */
    LoopClosure,                                          /**< correspondence information */
    AirPressure, AirPressureDiff,                         /**< barometric pressure */
    IMU,                                                  /**< IM [Acc, Gyr] */
    IterationSummary,                                     /**< timing of the optimizer */
    Error1, Error2, Error3, Error6,                       /**< residuals of cost functions */
    Cost, CostGradient1, CostGradient2, CostGradient3,    /**< cost of the optimizer */
    Value1,                                               /**< generic vector */
  };

  enum class DataElement  {Timestamp, TimestampRef,
                           Mean, Covariance, CovarianceDiagonal,
                           SatPos, SatElevation, SNR, SatID, SatSys,
                           Similarity,
                           Cost, Gradient, Hessian,
                           Weight,
                           WheelBase, TF, Velocity,
                           Other,
                           ID, BoxConf, Idx, BoxWLH, BoxAngle, BoxQuat, BoxClass, Key,
                           DurationSolver, DurationCovariance, DurationMarginal, DurationAdaptive, DurationTotal,
                           IterationSolver, IterationAdaptive};

  /** store the configuration of each data type in a global variable */
  using DataTypeConfig = DataConfig<DataType, DataElement>;
  extern const DataTypeConfig GlobalDataConfig;

  /** print enums */
  std::ostream& operator << (std::ostream& Os, const DataType& Type);
  std::ostream& operator << (std::ostream& Os, const FactorType& Type);
  std::ostream& operator << (std::ostream& Os, const SolutionType& Type);
  std::ostream& operator << (std::ostream& Os, const ErrorModelType& Type);
  std::ostream& operator << (std::ostream& Os, const ErrorModelTuningType& Type);

  /** dictionary to translate factor types into related sensor types */
  const std::map<FactorType, DataType> FactorSensorDict =
  {
    {FactorType::Range2, DataType::Range2},
    {FactorType::Range3, DataType::Range3},
    {FactorType::RangeToPoint2, DataType::Range2},
    {FactorType::RangeToPoint3, DataType::Range3},
    {FactorType::BetweenPose2, DataType::Pose2},
    {FactorType::BetweenPose3, DataType::Pose3},
    {FactorType::Point2Reg, DataType::Point2Set},
    {FactorType::Pseudorange2, DataType::Pseudorange2},
    {FactorType::Pseudorange3, DataType::Pseudorange3},
    {FactorType::Pseudorange3_Bias, DataType::Pseudorange3},
    {FactorType::Pseudorange3_ECEF, DataType::Pseudorange3},
    {FactorType::IMUPretintegration, DataType::IMU},
    {FactorType::IMUSimple, DataType::IMU},
    {FactorType::Odom2, DataType::Odom2},
    {FactorType::Odom2Diff, DataType::Odom2Diff},
    {FactorType::Odom4_ECEF, DataType::Odom3},
    {FactorType::Odom4, DataType::Odom3},
    {FactorType::Odom6, DataType::Odom3},
    {FactorType::Loop1, DataType::LoopClosure},
    {FactorType::Loop2, DataType::LoopClosure},
    {FactorType::Loop3, DataType::LoopClosure},
    {FactorType::LoopPose2, DataType::LoopClosure}
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
    {"const_quat",FactorType::ConstQuaternion},
    {"between_val1",FactorType::BetweenValue1},
    {"between_val2",FactorType::BetweenValue2},
    {"between_val3",FactorType::BetweenValue3},
    {"between_quat",FactorType::BetweenQuaternion},
    {"between_pose2",FactorType::BetweenPose2},
    {"between_pose3",FactorType::BetweenPose3},
    {"between_bearing_range2",FactorType::BetweenBearingRange2},
    {"range2",FactorType::Range2},
    {"range3",FactorType::Range3},
    {"range_point2",FactorType::RangeToPoint2},
    {"range_point3",FactorType::RangeToPoint3},
    {"pseudorange2",FactorType::Pseudorange2},
    {"pseudorange3",FactorType::Pseudorange3},
    {"pseudorange3_bias",FactorType::Pseudorange3_Bias},
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
    {"point2_reg",FactorType::Point2Reg},
    {"pressure_diff2",FactorType::PressureDiff2},
    {"pressure_diff3",FactorType::PressureDiff3},
    {"loop1",FactorType::Loop1},
    {"loop2",FactorType::Loop2},
    {"loop3",FactorType::Loop3},
    {"loop_pose2",FactorType::LoopPose2}
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
    {"prior",AbstractFactorType::Prior},
    {"pressure",AbstractFactorType::Pressure}
  };

  const std::map<std::string, ErrorModelType> ErrorModelTypeDict =
  {
    {"gauss",ErrorModelType::Gaussian},
    {"sc",ErrorModelType::SC},
    {"dcs",ErrorModelType::DCS},
    {"sc1",ErrorModelType::SC},
    {"dcs1",ErrorModelType::DCS},
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
    {"vbi_full",ErrorModelTuningType::VBI_Full},
    {"none",ErrorModelTuningType::None}
  };

  const std::map<std::string, SolutionType> SolutionTypeDict =
  {
    {"batch",SolutionType::Batch},
    {"smoother",SolutionType::Smoother},
    {"smoother_rt",SolutionType::SmootherRT},
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
      std::cerr << "Error in Types.h | Line 256 | TranslateSafe(): Dictionary entry is missing: " << Original;
      return false;
    }
    return true;
  }
}

#endif // TYPES_H
