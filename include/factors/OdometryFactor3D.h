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

/**
 * @file OdometryFactor3D.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Factors that connect two 3D poses with an odometry measurement. Different degrees of freedom are possible.
 * @copyright GNU Public License.
 *
 */

#ifndef ODOMETRYFACTOR3D_H
#define ODOMETRYFACTOR3D_H

#include <ceres/ceres.h>
#include "MeasurementFactor.h"
#include "../Geometry.h"
#include "../QuaternionCalc.h"

namespace libRSF
{
  template <int DOF>
  class OdometryModel3D : public SensorModel
  {
    public:
      OdometryModel3D()
      {
        _InputDim = 7;
        _OutputDim = DOF;
      }
  };

  template <>
  class OdometryModel3D<4> : public SensorModel
  {
    public:

      OdometryModel3D<4>()
      {
        _InputDim = 7;
        _OutputDim = 4;
      }

      template <typename T>
      bool Evaluate(const T* const Pos1, const T* const Pos2, const T* const Yaw1, const T* const Yaw2,
                    const ceres::Vector Odometry, const ceres::Vector DeltaTime,  T* Error) const
      {
        T PositionError[3];
        T PositionErrorBody[3];
        T QuatOldInv [4];
        T QuatPose [4];
        T ZAxis[3], QuatOld[4], QuatYaw[4];

        Eigen::Matrix<T, 4, 1> ErrorVector, Displacement;

        /** construct quaternion */
        ZAxis[0] = T(0.0);
        ZAxis[1] = T(0.0);
        ZAxis[2] = T(1.0);

        RotationBetween(ZAxis, Pos1, QuatPose);

        QuatYaw[0] = ceres::cos(Yaw1[0] / 2.0);
        QuatYaw[1] = T(0.0);
        QuatYaw[2] = T(0.0);
        QuatYaw[3] = ceres::sin(Yaw1[0] / 2.0);

        ceres::QuaternionProduct(QuatPose, QuatYaw, QuatOld);

        /** invert quaternion */
        libRSF::InvertQuaterion(QuatOld, QuatOldInv);

        /** calculate translation */
        PositionError[0] = Pos2[0] - Pos1[0];
        PositionError[1] = Pos2[1] - Pos1[1];
        PositionError[2] = Pos2[2] - Pos1[2];

        /** rotate translation back in body frame */
        ceres::QuaternionRotatePoint(QuatOldInv, PositionError, PositionErrorBody);

        /** weight position error */
        Displacement[0] = PositionErrorBody[0] ;
        Displacement[1] = PositionErrorBody[1] ;
        Displacement[2] = PositionErrorBody[2] ;

        /** calculate rotation error rotation */
        Displacement[3] = NormalizeAngle(Yaw2[0] - Yaw1[0]);

        /** convert to velocities and subtract measurement */
        ErrorVector = Displacement / T(DeltaTime[0]);
        ErrorVector.head(3) -= Odometry.head(3).template cast<T>();
        ErrorVector.tail(1) -= Odometry.tail(1).template cast<T>();

        Error[0] = ErrorVector[0];
        Error[1] = ErrorVector[1];
        Error[2] = ErrorVector[2];
        Error[3] = ErrorVector[3];

        return true;
      }
  };


  template <typename ErrorType>
  class OdometryFactor3D4DOF : public MeasurementFactor<OdometryModel3D<4>, ErrorType, 3, 3, 1, 1>
  {
    public:
      OdometryFactor3D4DOF(ErrorType &Error, MeasurementList &Measurements)
      {
        this->_Error = Error;

        this->_MeasurementVector.resize(7);
        this->_MeasurementVector.head(6) = Measurements.get(SensorType::Odom3).getMean();
        this->_MeasurementVector.tail(1) = Measurements.get(SensorType::DeltaTime).getMean();

        this->CheckInput();
      }

      /** normal version for static error models */
      template <typename T>
      bool operator()(const T* const Pos1, const T* const Pos2, const T* const Yaw1, const T* const Yaw2,  T* Error) const
      {
        this->_Model.Evaluate(Pos1, Pos2, Yaw1, Yaw2, this->_MeasurementVector.head(6), this->_MeasurementVector.tail(1), Error);
        this->_Error.Evaluate(Error);

        return true;
      }

      /** special version for SC and DCE */
      template <typename T>
      bool operator()(const T* const Pos1, const T* const Pos2, const T* const Yaw1, const T* const Yaw2, const T* const ErrorModelState,  T* Error) const
      {
        this->_Model.Evaluate(Pos1, Pos2, Yaw1, Yaw2, this->_MeasurementVector.head(6), this->_MeasurementVector.tail(1), Error);
        this->_Error.Evaluate(ErrorModelState, Error);

        return true;
      }
  };
}


#endif // ODOMETRYFACTOR3D_H
