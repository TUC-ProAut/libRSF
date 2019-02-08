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
 * @file OdometryFactor2DDifferential.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two 2D poses with a left/right-velocity and Y-velocity measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef ODOMETRYFACTOR2DDIFFERENTIAL_H
#define ODOMETRYFACTOR2DDIFFERENTIAL_H

#include <ceres/ceres.h>
#include "MeasurementFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  class OdometryModel2DDiffenrential : public SensorModel
  {
    public:
      OdometryModel2DDiffenrential()
      {
        _InputDim = 5;
        _OutputDim = 3;
      }

      template <typename T>
      bool Evaluate(const T* const Pos1, const T* const Pos2, const T* const Yaw1, const T* const Yaw2,
                    const ceres::Vector &Odometry, const ceres::Vector &DeltaTime, const ceres::Vector &WheelBase,  T* Error) const
      {
        /** calculate Error */
        Eigen::Matrix<T, 3, 1> Displacement, ErrorVector;
        Displacement = RelativeMotion2D(Pos1, Pos2, Yaw1, Yaw2);

        /** tranform in kinematic space */
        Eigen::Matrix<T, 2, 3> DifferentialTransformation;
        DifferentialTransformation << T(1.0), T(0.0), T(-WheelBase[0]),
                                   T(1.0), T(0.0), T(WheelBase[0]);
        Eigen::Matrix<T, 2, 1> Wheels = DifferentialTransformation * Displacement;


        /** calc and weight error */
        ErrorVector[0] = Wheels(0, 0);
        ErrorVector[1] = Wheels(1, 0);
        ErrorVector[2] = Displacement(1, 0);

        ErrorVector /= T(DeltaTime[0]);
        ErrorVector -= Odometry.template cast<T>();

        Error[0] = ErrorVector[0]; /** Left */
        Error[1] = ErrorVector[1]; /** Right */
        Error[2] = ErrorVector[2]; /** y motion */

        return true;
      }
  };

  template <typename ErrorType>
  class OdometryFactor2DDifferential : public MeasurementFactor<OdometryModel2DDiffenrential, ErrorType, 2, 2, 1, 1>
  {
    public:
      OdometryFactor2DDifferential(ErrorType &Error, MeasurementList &Measurements)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(5);
        this->_MeasurementVector.head(3) = Measurements.get(SensorType::Odom2Diff).getMean();
        this->_MeasurementVector.segment(3, 1) = Measurements.get(SensorType::Odom2Diff).getValue(SensorElement::WheelBase);
        this->_MeasurementVector.tail(1) = Measurements.get(SensorType::DeltaTime).getMean();

        this->CheckInput();
      }

      /** normal version for static error models */
      template <typename T>
      bool operator()(const T* const Pos1, const T* const Pos2, const T* const Yaw1, const T* const Yaw2,  T* Error) const
      {
        this->_Model.Evaluate(Pos1, Pos2, Yaw1, Yaw2, this->_MeasurementVector.head(3), this->_MeasurementVector.tail(1), this->_MeasurementVector.segment(3, 1), Error);
        this->_Error.Evaluate(Error);

        return true;
      }

      /** special version for SC and DCE */
      template <typename T>
      bool operator()(const T* const Pos1, const T* const Pos2, const T* const Yaw1, const T* const Yaw2, const T* const ErrorModelState,  T* Error) const
      {
        this->_Model.Evaluate(Pos1, Pos2, Yaw1, Yaw2, this->_MeasurementVector.head(3), this->_MeasurementVector.tail(1), this->_MeasurementVector.segment(3,1), Error);
        this->_Error.Evaluate(ErrorModelState, Error);

        return true;
      }
  };
}


#endif // ODOMETRYFACTOR2DDIFFERENTIAL_H
