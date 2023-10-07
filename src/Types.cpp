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

#include "Types.h"


namespace libRSF
{
  std::ostream &operator << (std::ostream& Os, const DataType& Type)
  {
    Os <<  static_cast<int>(Type);
    return Os;
  }

  std::ostream& operator << (std::ostream& Os, const FactorType& Type)
  {
    for (const auto &Entry : FactorTypeDict)
    {
      if(Entry.second == Type)
      {
        Os << Entry.first;
      }
    }
    return Os;
  }

  std::ostream& operator << (std::ostream& Os, const ErrorModelType& Type)
  {
    for (const auto &Entry : ErrorModelTypeDict)
    {
      if(Entry.second == Type)
      {
        Os << Entry.first;
      }
    }
    return Os;
  }

  std::ostream& operator << (std::ostream& Os, const ErrorModelTuningType& Type)
  {
    for (const auto &Entry : ErrorModelTuningTypeDict)
    {
      if(Entry.second == Type)
      {
        Os << Entry.first;
      }
    }
    return Os;
  }

  std::ostream &operator << (std::ostream& Os, const SolutionType& Type)
  {
    /** print numeric value */
    Os <<  static_cast<int>(Type);
    return Os;
  }

   /** define states with identifier */
  const DataTypeConfig::InitVect DataConfigInit =
  {
    /** pure vectors for translation */
    {
      "point1", DataType::Point1,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    },
    {
      "point2", DataType::Point2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::Covariance, 4}
      }
    },
    {
      "point3", DataType::Point3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3},
        {DataElement::Covariance, 9}
      }
    },

    /** rotation representations */
    {
      "angle", DataType::Angle,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    },
    {
      "quaternion", DataType::Quaternion,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 4}, /**< [x, y, z, w] */
        {DataElement::Covariance, 16}
      }
    },
    {
      "unit_circle", DataType::UnitCircle,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2}, /**< [cos(alpha), sin(alpha)] */
        {DataElement::Covariance, 4}
      }
    },

    /** joint poses */
    {
      "pose2", DataType::Pose2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3}, /**< [x, y, yaw] */
        {DataElement::Covariance, 9}
      }
    },
    {
      "pose3", DataType::Pose3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 7}, /**< Trans[x, y, z], Quat[x, y, z, w] */
        {DataElement::Covariance, 36} /**< in tangent space this is just 6x6 */
      }
    },

    /** joint poses */
    {
      "pose_between2", DataType::PoseBetween2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::TimestampRef, 1},
        {DataElement::Mean, 3}, /**< [x, y, yaw] */
        {DataElement::Covariance, 9}
      }
    },
    {
      "pose_between3", DataType::PoseBetween3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::TimestampRef, 1},
        {DataElement::Mean, 7}, /**< Trans[x, y, z], Quat[x, y, z, w] */
        {DataElement::Covariance, 36} /**< in tangent space this is just 6x6 */
      }
    },

    /** position with a specific ID */
    {
      "point_id2", DataType::PointID2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::ID, 1},
        {DataElement::Covariance, 4}
      }
    },

    /** position with ID and confidence for tracking*/
    {
      "point_conf_id2", DataType::PointConfID2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::ID, 1},
        {DataElement::Idx, 1},
        {DataElement::BoxConf, 1},
        {DataElement::Covariance, 4}
      }
    },

    /** 3D bounding box for tracking */
    {
      "bounding_box_3", DataType::BoundingBox3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3},
        {DataElement::Velocity, 3},
        {DataElement::ID, 1},
        {DataElement::Idx, 1},
        {DataElement::BoxConf, 1},
        {DataElement::Covariance, 9},
        {DataElement::BoxWLH, 3},
        {DataElement::BoxAngle, 1},
        {DataElement::BoxQuat, 4},
        {DataElement::BoxClass, 1},
        {DataElement::Key, 1}
      }
    },

    /**< GNSS receiver clock error */
    {
      "clock", DataType::ClockError,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    },
    {
      "clock_drift", DataType::ClockDrift,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    },

    /** IMU speed and bias */
    {
      "imu_bias", DataType::IMUBias,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 9}, /**< Speed, Acc Bias, TR Bias */
        {DataElement::Covariance, 81}
      }
    },

    /** dynamic error model variables */
    {
      "switch", DataType::Switch,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    },
    {
      "cov1", DataType::Covariance1,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    },
    {
      "cov2", DataType::Covariance2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::Covariance, 4}
      }
    },

    /** Gaussian mixtures */
    {
      "gmm1", DataType::GMM1,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1},
        {DataElement::Weight, 1}
      }
    },
    {
        "gmm2", DataType::GMM2,
        {
            {DataElement::Timestamp, 1},
            {DataElement::Mean, 2},
            {DataElement::Covariance, 4},
            {DataElement::Weight, 1}
        }
    },

    /** odometry */
    {
      "odom2", DataType::Odom2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3},
        {DataElement::CovarianceDiagonal , 3}
      }
    },

    {
      "odom2diff", DataType::Odom2Diff,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3},
        {DataElement::WheelBase, 1},
        {DataElement::CovarianceDiagonal , 3}
      }
    },

    {
      "odom3", DataType::Odom3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6},
        {DataElement::CovarianceDiagonal , 6}
      }
    },

    /** relative information from different sensors */
    {
      "odom3radar", DataType::Odom3Radar,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6},
        {DataElement::CovarianceDiagonal , 6}
      }
    },
    {
      "odom3laser", DataType::Odom3Laser,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6},
        {DataElement::CovarianceDiagonal , 6}
      }
    },
    {
      "odom3vio", DataType::Odom3VIO,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6},
        {DataElement::CovarianceDiagonal , 6}
      }
    },

    /** IMU */
    {
      "imu", DataType::IMU,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6}, /**< [Acc, TR] */
        {DataElement::CovarianceDiagonal , 6}
      }
    },

    /** GNSS */
    {
      "pseudorange2", DataType::Pseudorange2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance , 1},
        {DataElement::SatPos, 2},
        {DataElement::SatID , 1}
      }
    },
    {
      "pseudorange3", DataType::Pseudorange3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance , 1},
        {DataElement::SatPos, 3},
        {DataElement::SatID , 1},
        {DataElement::SatSys , 1},
        {DataElement::SatElevation , 1},
        {DataElement::SNR , 1}
      }
    },

    /** ranging */
    {
      "range2", DataType::Range2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance , 1},
        {DataElement::SatPos, 2},
        {DataElement::SatID , 1},
            {DataElement::SNR , 1}
      }
    },
    {
      "range3", DataType::Range3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance , 1},
        {DataElement::SatPos, 3},
        {DataElement::SatID , 1},
        {DataElement::SNR , 1}
      }
    },

    /** range SLAM */
    {
        "range_lm2", DataType::RangeLM2,
        {
            {DataElement::Timestamp, 1},
            {DataElement::Mean, 1},
            {DataElement::Covariance , 1},
            {DataElement::SatID , 1},
            {DataElement::SNR , 1}
        }
    },
    {
        "range_lm3", DataType::RangeLM3,
        {
            {DataElement::Timestamp, 1},
            {DataElement::Mean, 1},
            {DataElement::Covariance , 1},
            {DataElement::SatID , 1},
            {DataElement::SNR , 1}
        }
    },

    /** points from radar sensor */
    {
      "point2radar", DataType::Point2Radar,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3},/** distance, angle, doppler velocity */
        {DataElement::Covariance , 9}
      }
    },

    /** absolute measurements: point correspondence */
    {
      "point1set", DataType::Point1Set,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::Covariance, 4}
      }
    },
    {
      "point2set", DataType::Point2Set,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 4},
        {DataElement::Covariance, 8}
      }
    },
    {
      "point3set", DataType::Point3Set,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6},
        {DataElement::Covariance, 18}
      }
    },

    {
      "point2setradar", DataType::Point2SetRadar,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6},
        {DataElement::Covariance, 18}
      }
    },

    /** relative bearing range */
    {
      "bearing_range_id_2", DataType::BearingRangeID2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::CovarianceDiagonal, 2},
        {DataElement::ID, 1}
      }
    },

    /** barometric pressure */
    {
      "pressure", DataType::AirPressure,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance , 1}
      }
    },
    {
      "pressure_diff", DataType::AirPressureDiff,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance , 1}
      }
    },

    /** loop closures */
    {
      "loop", DataType::LoopClosure,
      {
        {DataElement::Timestamp, 1},
        {DataElement::TimestampRef, 1},
        {DataElement::Similarity, 1}
      }
    },

    /** plain factor error */
    {
      "error1", DataType::Error1,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1}
      }
    },
    {
      "error2", DataType::Error2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2}
      }
    },
    {
      "error3", DataType::Error3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3}
      }
    },
    {
      "error6", DataType::Error6,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 6}
      }
    },

    /** cost surface */
    {
      "cost", DataType::Cost,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Cost, 1}
      }
    },
    {
      "cost_gradient1", DataType::CostGradient1,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Cost, 1},
        {DataElement::Gradient, 1},
        {DataElement::Hessian, 1}
      }
    },
    {
      "cost_gradient2", DataType::CostGradient2,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 2},
        {DataElement::Cost, 1},
        {DataElement::Gradient, 2},
        {DataElement::Hessian, 4}
      }
    },
    {
      "cost_gradient3", DataType::CostGradient3,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 3},
        {DataElement::Cost, 1},
        {DataElement::Gradient, 3},
        {DataElement::Hessian, 9}
      }
    },

    /** runtime of different steps */
    {
      "solver_summary", DataType::IterationSummary,
      {
        {DataElement::Timestamp, 1},
        {DataElement::DurationTotal, 1},
        {DataElement::DurationSolver, 1},
        {DataElement::DurationCovariance, 1},
        {DataElement::DurationMarginal, 1},
        {DataElement::DurationAdaptive, 1},
        {DataElement::IterationSolver, 1},
        {DataElement::IterationAdaptive, 1}
      }
    },

    /** generic 1D value for diverse purpose */
    {
      "val1", DataType::Value1,
      {
        {DataElement::Timestamp, 1},
        {DataElement::Mean, 1},
        {DataElement::Covariance, 1}
      }
    }
  };

  /** init the config based on the given list */
  const DataTypeConfig GlobalDataConfig(DataConfigInit);
}
