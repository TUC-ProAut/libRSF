/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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

#include "SensorData.h"

namespace libRSF
{
  /** define states with identifier */
  SensorConfig::InitVect SensorConfigInit =
  {
    /** odometry */
    {
      "odom2", SensorType::Odom2,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 3},
        {SensorElement::Covariance , 3}
      }
    },

    {
      "odom2diff", SensorType::Odom2Diff,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 3},
        {SensorElement::WheelBase, 1},
        {SensorElement::Covariance , 3}
      }
    },

    {
      "odom3", SensorType::Odom3,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6},
        {SensorElement::Covariance , 6}
      }
    },

    /** relative information from different sensors */
    {
      "odom3radar", SensorType::Odom3Radar,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6},
        {SensorElement::Covariance , 6}
      }
    },
    {
      "odom3laser", SensorType::Odom3Laser,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6},
        {SensorElement::Covariance , 6}
      }
    },
    {
      "odom3vio", SensorType::Odom3VIO,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6},
        {SensorElement::Covariance , 6}
      }
    },

    /** IMU */
    {
      "imu", SensorType::IMU,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6}, /**< [Acc, TR] */
        {SensorElement::Covariance , 6}
      }
    },

    /** GNSS */
    {
      "pseudorange2", SensorType::Pseudorange2,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1},
        {SensorElement::SatPos, 2},
        {SensorElement::SatID , 1}
      }
    },
    {
      "pseudorange3", SensorType::Pseudorange3,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1},
        {SensorElement::SatPos, 3},
        {SensorElement::SatID , 1},
        {SensorElement::SatElevation , 1},
        {SensorElement::SNR , 1}
      }
    },

    /** ranging */
    {
      "range2", SensorType::Range2,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1},
        {SensorElement::SatPos, 2},
        {SensorElement::SatID , 1}
      }
    },
    {
      "range3", SensorType::Range3,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1},
        {SensorElement::SatPos, 3},
        {SensorElement::SatID , 1},
        {SensorElement::SNR , 1}
      }
    },

    /** absolute measurements: points */
    {
      "point1", SensorType::Point1,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1}
      }
    },
    {
      "point2", SensorType::Point2,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 2},
        {SensorElement::Covariance , 4}
      }
    },
    {
      "point3", SensorType::Point3,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 3},
        {SensorElement::Covariance , 9}
      }
    },

    {
      "point_id2", SensorType::PointID2,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 2},
        {SensorElement::ID, 1},
        {SensorElement::Idx, 1},
        {SensorElement::Conf, 1},
        {SensorElement::Covariance, 4}
      }
    },
    {
      "point_id3", SensorType::PointID3,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 3},
        {SensorElement::Velocity, 3},
        {SensorElement::ID, 1},
        {SensorElement::Idx, 1},
        {SensorElement::Conf, 1},
        {SensorElement::Covariance, 9}
      }
    },

    {
      "point2radar", SensorType::Point2Radar,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 3},/** distance, angle, doppler velocity */
        {SensorElement::Covariance , 3*3}
      }
    },

    /** absolute measurements: point correspondence */
    {
      "point1set", SensorType::Point1Set,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 2},
        {SensorElement::Covariance, 4}
      }
    },
    {
      "point2set", SensorType::Point2Set,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 4},
        {SensorElement::Covariance, 8}
      }
    },
    {
      "point3set", SensorType::Point3Set,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6},
        {SensorElement::Covariance, 18}
      }
    },

    {
      "point2setradar", SensorType::Point2SetRadar,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 6},
        {SensorElement::Covariance, 18}
      }
    },

    /** absolute measurements: poses */
    {
      "pose2", SensorType::Pose2,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 3},
        {SensorElement::Covariance , 3*3}
      }
    },

    {
      "pose3", SensorType::Pose3,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 7}, /**< Trans[x, y, z], Quat[x, y, z, w] */
        {SensorElement::Covariance , 6*6} /**< in tangent space this is just 6x6 */
      }
    },

    /** absolute measurements: rotations */
    {
      "angle", SensorType::Angle,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1}
      }
    },

    {
      "quat", SensorType::Quaternion,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 4}, /**< [x, y, z, w] */
        {SensorElement::Covariance , 9} /**< in tangent space this is just 3x3 */
      }
    },

    /** loop closures */
    {
      "loop", SensorType::LoopClosure,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::TimestampRef, 1},
      }
    },

    /** dummy measurements */
    {
      "val1", SensorType::Val1,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1}
      }
    },

    {
      "other", SensorType::Other,
      {
        {SensorElement::Timestamp, 1},
        {SensorElement::Mean, 1},
        {SensorElement::Covariance , 1}
      }
    }
  };

  /** create config based in the initializer list */
  const SensorConfig Sensors(SensorConfigInit);

  SensorData::SensorData()
  {
    this->_Config = &Sensors;
  }

  SensorData::SensorData(std::string Input)
  {
    this->_Config = &Sensors;
    this->constructFromString(Input);
  }

  SensorData::SensorData(SensorType Type, double Timestamp)
  {
    this->_Config = &Sensors;
    this->constructEmpty(Type, Timestamp);
  }
}
