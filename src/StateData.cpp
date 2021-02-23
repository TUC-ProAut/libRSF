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

#include "StateData.h"

namespace libRSF
{
  /** define states with identifier */
  const StateConfig::InitVect StateConfigInit =
  {
    /** position */
    {
      "point1", StateType::Point1,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1}
      }
    },

    {
      "point2", StateType::Point2,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 2},
        {StateElement::Covariance, 4}
      }
    },

    {
      "point3", StateType::Point3,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 3},
        {StateElement::Covariance, 9}
      }
    },

    {
      "unit_circle", StateType::UnitCircle,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 2},
        {StateElement::Covariance, 4}
      }
    },

    {
      "point_id2", StateType::PointID2,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 2},
        {StateElement::ID, 1},
        {StateElement::Idx, 1},
        {StateElement::Conf, 1},
        {StateElement::Covariance, 4}
      }
    },

    {
      "point_id3", StateType::PointID3,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 3},
        {StateElement::ID, 1},
        {StateElement::Idx, 1},
        {StateElement::Conf, 1},
        {StateElement::Covariance, 9},
        {StateElement::WLH, 3},
        {StateElement::R, 1},
        {StateElement::R_Quat, 4},
        {StateElement::Class, 1},
        {StateElement::Key, 1},
        {StateElement::Velocity, 3}
      }
    },

    /** orientation */
    {
      "angle", StateType::Angle,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1}
      }
    },

    {
      "quaternion", StateType::Quat,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 4}, /**< [x, y, z, w] */
        {StateElement::Covariance, 4*4}
      }
    },

    /** combined pose */
    {
      "pose2", StateType::Pose2,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 3}, /**< [x, y, yaw] */
        {StateElement::Covariance, 3*3}
      }
    },

    {
      "pose3", StateType::Pose3,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 7}, /**< [tx, ty, tz, qx, qy, qz, qw] */
        {StateElement::Covariance, 7*7}
      }
    },

    /** IMU speed and bias */
    {
      "imu_bias", StateType::IMUBias,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 9}, /**< Speed, Acc Bias, TR Bias */
        {StateElement::Covariance, 9*9}
      }
    },

    /** GNSS clock */
    {
      "clock", StateType::ClockError,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1}
      }
    },

    {
      "clock_drift", StateType::ClockDrift,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1}
      }
    },

    /** error model variables */
    {
      "switch", StateType::Switch,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1}
      }
    },

    {
      "cov1", StateType::Covariance1,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1}
      }
    },

    /** gaussian mixtures */
    {
      "gmm", StateType::GMM,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Covariance, 1},
        {StateElement::Weight, 1}
      }
    },

    /** plain factor error */
    {
      "error1", StateType::Error1,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1}
      }
    },

    {
      "error2", StateType::Error2,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 2}
      }
    },

    {
      "error3", StateType::Error3,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 3}
      }
    },

    {
      "error6", StateType::Error6,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 6}
      }
    },

    /** cost surface */
    {
      "cost", StateType::Cost,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Cost, 1}
      }
    },

    {
      "cost_gradient", StateType::CostGradient,
      {
        {StateElement::Timestamp, 1},
        {StateElement::Mean, 1},
        {StateElement::Cost, 1},
        {StateElement::Gradient, 1},
        {StateElement::Hessian, 1}
      }
    },

    /** runtime of different steps */
    {
      "solver_summary", StateType::IterationSummary,
      {
        {StateElement::Timestamp, 1},
        {StateElement::DurationTotal, 1},
        {StateElement::DurationSolver, 1},
        {StateElement::DurationMarginal, 1},
        {StateElement::DurationAdaptive, 1},
        {StateElement::IterationSolver, 1},
        {StateElement::IterationAdaptive, 1}
      }
    }

  };

  const StateConfig States(StateConfigInit);

  StateData::StateData()
  {
    this->_Config = &States;
  }

  StateData::StateData(std::string Input)
  {
    this->_Config = &States;
    this->constructFromString(Input);
  }

  StateData::StateData(StateType Type, double Timestamp)
  {
    this->_Config = &States;
    this->constructEmpty(Type, Timestamp);
  }
}
