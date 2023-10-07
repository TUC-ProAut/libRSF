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
 * @file IMUModel.h
 * @author Tim Pfeifer
 * @date 11.05.2020
 * @brief Implementation of geometric IMU models.
 * @copyright GNU Public License.
 *
 */

#ifndef IMUMODEL_H
#define IMUMODEL_H

#include "../Geometry.h"
#include "../Constants.h"

//#define USE_CENTRIPETAL
#define CENTRIPETAL_SIGN T(1.0)

namespace libRSF
{
  template<typename T>
  class IMUModel
  {
    public:
      IMUModel() = default;
      virtual ~IMUModel() = default;

      void static inline applyForward(const T* Point1Ptr,
                                      const T* Quat1Ptr,
                                      const T* SpedBias1Ptr,
                                      T* Point2Ptr,
                                      T* Quat2Ptr,
                                      T* SpedBias2Ptr,
                                      const VectorT<T, 3> Acceleration,
                                      const VectorT<T, 3> TurnRate,
                                      const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRef<T,3>      Point2(Point2Ptr);

        QuaternionRefConst<T> Quat1(Quat1Ptr);
        QuaternionRef<T>      Quat2(Quat2Ptr);

        VectorRefConst<T,3> Velocity1(SpedBias1Ptr);
        VectorRef<T,3>      Velocity2(SpedBias2Ptr);
        VectorRefConst<T,3> AccBias1(SpedBias1Ptr+3);
        VectorRef<T,3>      AccBias2(SpedBias2Ptr+3);
        VectorRefConst<T,3> GyroBias1(SpedBias1Ptr+6);
        VectorRef<T,3>      GyroBias2(SpedBias2Ptr+6);

        /** unbiased measurements */
        VectorT<T, 3> AccTrue = Acceleration - AccBias1;
        const VectorT<T, 3> TRTrue = TurnRate - GyroBias1;

        /** correction of centripetal force */
        #ifdef USE_CENTRIPETAL
          AccTrue += TRTrue.cross(Quat1.conjugate() * Velocity1)*CENTRIPETAL_SIGN;
        #endif // USE_CENTRIPETAL

        /** acceleration in world coordinates */
        const VectorT<T, 3> AccWorld = (Quat1 * AccTrue) - GRAVITY_VECTOR;

        /** apply acceleration */
        Velocity2 = Velocity1 + AccWorld*DeltaTime;

        /** apply translation */
        Point2 = Point1 + Velocity1*DeltaTime + 0.5*AccWorld*DeltaTime*DeltaTime;

        /** apply rotation */
        Quat2 = Quat1 * AngularVelocityToQuaternion<T>(TRTrue, DeltaTime);

        /** constant biases */
        AccBias2 = AccBias1;
        GyroBias2 = GyroBias1;
      }

      void static inline applyBackward(const T* Point1Ptr,
                                       const T* Quat1Ptr,
                                       const T* SpedBias1Ptr,
                                       const T* Point2Ptr,
                                       const T* Quat2Ptr,
                                       const T* SpedBias2Ptr,
                                       VectorT<T, 3> &Velocity,
                                       VectorT<T, 3> &Acceleration,
                                       VectorT<T, 3> &TurnRate,
                                       const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRefConst<T,3> Point2(Point2Ptr);

        QuaternionRefConst<T> Quat1(Quat1Ptr);
        QuaternionRefConst<T> Quat2(Quat2Ptr);

        VectorRefConst<T,3> Velocity1(SpedBias1Ptr);
        VectorRefConst<T,3> Velocity2(SpedBias2Ptr);
        VectorRefConst<T,3> AccBias1(SpedBias1Ptr+3);
        VectorRefConst<T,3> GyroBias1(SpedBias1Ptr+6);

        /** estimate rotation */
        TurnRate = QuaternionToAngularVelocity<T>(Quat1.conjugate()*Quat2, DeltaTime);

        /** estimate velocity (in world frame) */
        Velocity = (Point2 - Point1) / DeltaTime;

        /** estimate acceleration (in body frame) */
        Acceleration = Quat1.conjugate() * (Velocity2 - Velocity1) / DeltaTime;

        /** correction of centripetal force */
        #ifdef USE_CENTRIPETAL
          Acceleration -= TurnRate.cross(Quat1.conjugate() * Velocity1)*CENTRIPETAL_SIGN;
        #endif // USE_CENTRIPETAL

        /** add biases and gravity*/
        TurnRate += GyroBias1;
        Acceleration += AccBias1 + Quat1.conjugate() * GRAVITY_VECTOR.template cast<T>();
      }
  };
}



#endif // IMUMODEL_H
