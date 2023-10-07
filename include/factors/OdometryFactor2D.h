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
 * @file OdometryFactor2D.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two 2D poses with a X/Y-velocity and turn rate measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef ODOMETRYFACTOR2D_H
#define ODOMETRYFACTOR2D_H

#include "BaseFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType>
  class OdometryFactor2D : public BaseFactor<ErrorType, true, true, 2, 1, 2, 1>
  {
    public:
      /** construct factor and store measurement */
      OdometryFactor2D(ErrorType &Error, const Data &OdometryMeasurement, double DeltaTime)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(3);
        this->MeasurementVector_ = OdometryMeasurement.getMean();
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 3> Evaluate(const T* const Point1,  const T* const Yaw1,
                             const T* const Point2,  const T* const Yaw2) const
      {
        VectorRefConst<T, 2> P1(Point1);
        VectorRefConst<T, 2> P2(Point2);

        const Rotation2DT<T> R1(Yaw1[0]);
        const Rotation2DT<T> R2(Yaw2[0]);

        VectorT<T, 3> Error;
        VectorT<T, 3> RelPose;

        /** translation */
        RelPose.head(2) = (R1.inverse() * (P2 - P1));

        RelPose = RelativeMotion2D(Point1, Yaw1, Point2, Yaw2);

        /** rotation */
        RelPose(2) = Yaw2[0] - Yaw1[0];

        /** subtract odometry */
        Error = RelPose - (this->DeltaTime_ * this->MeasurementVector_).template cast<T>();

        /** normalize angle error */
        Error(2) = NormalizeAngle<T>(Error(2));

        /** transform in measurement (velocity) domain */
        Error /= T(this->DeltaTime_);

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Point1,  const T* const Yaw1,
                      const T* const Point2,  const T* const Yaw2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Point1, Yaw1, Point2, Yaw2),
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

        /** map to rotation */
        const Rotation2DT<double> R1(Yaw1(0));
        const Rotation2DT<double> ROdom(this->MeasurementVector_(2) * this->DeltaTime_);

        P2 = P1 + R1 * this->MeasurementVector_.head(2) * this->DeltaTime_;

        Yaw2(0) = (ROdom * R1).angle();
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Odom2, ErrorType> {using Type = OdometryFactor2D<ErrorType>;};
}


#endif // ODOMETRYFACTOR2D_H
