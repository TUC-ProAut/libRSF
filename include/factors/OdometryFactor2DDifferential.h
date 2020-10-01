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

#include "BaseFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType>
  class OdometryFactor2DDifferential : public BaseFactor<ErrorType, true, true, 2, 2, 1, 1>
  {
    public:
      /** construct factor and store measurement */
      OdometryFactor2DDifferential(ErrorType &Error, const SensorData &OdometryMeasurement, double DeltaTime)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(4);
        this->_MeasurementVector.head(3) = OdometryMeasurement.getMean();
        this->_MeasurementVector.tail(1) = OdometryMeasurement.getValue(SensorElement::WheelBase);
        this->_DeltaTime = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 3> Evaluate(const T* const Pos1, const T* const Pos2,
                             const T* const Yaw1, const T* const Yaw2,
                             const Vector3 &Odometry, const double &DeltaTime, const double &WheelBase) const
      {
        /** calculate Error */
        VectorT<T, 3> Displacement, Error;
        Displacement = RelativeMotion2D(Pos1, Pos2, Yaw1, Yaw2);

        /** tranform in kinematic space */
        MatrixT<T, 2, 3> DifferentialTransformation;
        DifferentialTransformation << T(1.0), T(0.0), T(-WheelBase),
                                   T(1.0), T(0.0), T(WheelBase);
        VectorT<T, 2> Wheels = DifferentialTransformation * Displacement;

        /** calc and weight error */
        Error[0] = Wheels(0, 0);
        Error[1] = Wheels(1, 0);
        Error[2] = Displacement(1, 0);

        Error /= T(DeltaTime);
        Error -= Odometry.template cast<T>();

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Pos2,
                      const T* const Yaw1, const T* const Yaw2,
                      ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(Pos1, Pos2,
                                                              Yaw1, Yaw2,
                                                              this->_MeasurementVector.head(3),
                                                              this->_DeltaTime,
                                                              this->_MeasurementVector(3)),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Odom2Diff, ErrorType> {using Type = OdometryFactor2DDifferential<ErrorType>;};
}


#endif // ODOMETRYFACTOR2DDIFFERENTIAL_H
