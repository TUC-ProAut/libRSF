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
  class OdometryFactor2DDifferential : public BaseFactor<ErrorType, true, true, 2, 1, 2, 1>
  {
    public:
      /** construct factor and store measurement */
      OdometryFactor2DDifferential(ErrorType &Error, const Data &OdometryMeasurement, double DeltaTime)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(4);
        this->MeasurementVector_.head(3) = OdometryMeasurement.getMean();
        this->MeasurementVector_.tail(1) = OdometryMeasurement.getValue(DataElement::WheelBase);
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 3> Evaluate(const T* const Pos1, const T* const Yaw1,
                             const T* const Pos2, const T* const Yaw2) const
      {
        /** get odometry information */
        const double WheelBase = this->MeasurementVector_(3);
        const Vector3 Odometry = this->MeasurementVector_.head(3);

        /** calculate Error */
        VectorT<T, 3> Displacement, Error;
        Displacement = RelativeMotion2D(Pos1, Yaw1, Pos2, Yaw2);

        /** transform in kinematic space */
        MatrixT<T, 2, 3> DifferentialTransformation;
        DifferentialTransformation << T(1.0), T(0.0), T(-WheelBase),
                                   T(1.0), T(0.0), T(WheelBase);
        VectorT<T, 2> Wheels = DifferentialTransformation * Displacement;

        /** calc and weight error */
        Error[0] = Wheels(0, 0);
        Error[1] = Wheels(1, 0);
        Error[2] = Displacement(1, 0);

        Error /= T(this->DeltaTime_);
        Error -= Odometry.template cast<T>();

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Yaw1,
                      const T* const Pos2, const T* const Yaw2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Yaw1,
                                                              Pos2, Yaw2),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        /** map pointer to vectors */
        VectorRefConst<double, 2> P1(StatePointers.at(0));
        VectorRefConst<double, 1> Yaw1(StatePointers.at(1));
        VectorRef<double, 2>      P2(StatePointers.at(2));
        VectorRef<double, 1>      Yaw2(StatePointers.at(3));

        /** get odometry information */
        const double WheelBase = this->MeasurementVector_(3);
        const Vector3 Odometry = this->MeasurementVector_.head(3);

        /** predict translation */
        Vector2 POdom;
        POdom << (Odometry(1) + Odometry(0)) / 2.0, Odometry(2);
        POdom = RotationMatrix2D(Yaw1(0)) * POdom * this->DeltaTime_;
        P2 = P1 + POdom;

        /** predict rotation */
        Yaw2(0) = NormalizeAngle(Yaw1(0) + (Odometry(1) - Odometry(0)) / (WheelBase * 2.0) * this->DeltaTime_);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Odom2Diff, ErrorType> {using Type = OdometryFactor2DDifferential<ErrorType>;};
}


#endif // ODOMETRYFACTOR2DDIFFERENTIAL_H
