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
 * @file OdometryModel.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Implementation of geometric odometry models.
 * @copyright GNU Public License.
 *
 */

#ifndef ODOMETRYMODEL_H
#define ODOMETRYMODEL_H

#include "../Geometry.h"

namespace libRSF
{
  template<typename T>
  class OdometryModel6DOF
  {
    public:
      OdometryModel6DOF() = default;
      virtual ~OdometryModel6DOF() = default;

      void static inline applyForward(const T* Point1Ptr,
                                      const T* Quat1Ptr,
                                      T* Point2Ptr,
                                      T* Quat2Ptr,
                                      const VectorT<T, 3> Velocity,
                                      const VectorT<T, 3> TurnRate,
                                      const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRef<T,3>      Point2(Point2Ptr);
        QuaternionRefConst<T> Quat1(Quat1Ptr);
        QuaternionRef<T>      Quat2(Quat2Ptr);

        /** apply transformation */
        Point2 = Point1 + Quat1 * (Velocity * DeltaTime);
        Quat2 = Quat1 * AngularVelocityToQuaternion<T>(TurnRate, DeltaTime);
      }

      void static inline applyBackward(const T* Point1Ptr,
                                       const T* Quat1Ptr,
                                       const T* Point2Ptr,
                                       const T* Quat2Ptr,
                                       VectorT<T, 3> &Velocity,
                                       VectorT<T, 3> &TurnRate,
                                       const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRefConst<T,3> Point2(Point2Ptr);
        QuaternionRefConst<T> Quat1(Quat1Ptr);
        QuaternionRefConst<T> Quat2(Quat2Ptr);

        /** transform backward */
        Velocity = Quat1.conjugate() * (Point2 - Point1) / DeltaTime;
        TurnRate = QuaternionToAngularVelocity(Quat1.conjugate()*Quat2, DeltaTime);
      }
  };

  template<typename T>
  class OdometryModel4DOF
  {
    public:
      OdometryModel4DOF() = default;
      virtual ~OdometryModel4DOF() = default;

      void static inline applyForward(const T* Point1Ptr,
                                      const T* Yaw1Ptr,
                                      T* Point2Ptr,
                                      T* Yaw2Ptr,
                                      const VectorT<T, 3> Velocity,
                                      const VectorT<T, 3> TurnRate,
                                      const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRef<T,3> Point2(Point2Ptr);

        /** apply translation */
        Point2 = Point1 + AngleAxisT<T>(*Yaw1Ptr, VectorT<T,3>::UnitZ()) * (Velocity * DeltaTime);

        /** apply rotation */
        *Yaw2Ptr = NormalizeAngle<T>(*Yaw1Ptr + TurnRate(2)*DeltaTime);
      }

      void static inline applyBackward(const T* Point1Ptr,
                                       const T* Yaw1Ptr,
                                       const T* Point2Ptr,
                                       const T* Yaw2Ptr,
                                       VectorT<T, 3> &Velocity,
                                       VectorT<T, 3> &TurnRate,
                                       const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRefConst<T,3> Point2(Point2Ptr);

        /** estimate translation */
        Velocity = AngleAxisT<T>(-*Yaw1Ptr, VectorT<T,3>::UnitZ()) * (Point2 - Point1) / DeltaTime;

        /** estimate rotation */
        TurnRate.setZero();
        TurnRate(2) = NormalizeAngle<T>(*Yaw2Ptr - *Yaw1Ptr)/DeltaTime;
      }
  };

  template<typename T>
  class OdometryModel4DOFECEF
  {
    public:
      OdometryModel4DOFECEF() = default;
      virtual ~OdometryModel4DOFECEF() = default;

      void static inline applyForward(const T* Point1Ptr,
                                      const T* Yaw1Ptr,
                                      T* Point2Ptr,
                                      T* Yaw2Ptr,
                                      const VectorT<T, 3> Velocity,
                                      const VectorT<T, 3> TurnRate,
                                      const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRef<T,3> Point2(Point2Ptr);

        /** rotation from ECEF frame to a local frame and add yaw */
        const VectorT<T, 3> ZAxis = VectorT<T,3>::UnitZ();
        const QuaternionT<T> QuatWorldToBodyToOdom = QuaternionT<T>::FromTwoVectors(ZAxis, Point1) * AngleAxisT<T>(*Yaw1Ptr, VectorT<T,3>::UnitZ());

        /** apply translation */
        Point2 = Point1 + QuatWorldToBodyToOdom * (Velocity * DeltaTime);

        /** apply rotation */
        *Yaw2Ptr = NormalizeAngle<T>(*Yaw1Ptr + TurnRate(2)*DeltaTime);
      }

      void static inline applyBackward(const T* Point1Ptr,
                                       const T* Yaw1Ptr,
                                       const T* Point2Ptr,
                                       const T* Yaw2Ptr,
                                       VectorT<T, 3> &Velocity,
                                       VectorT<T, 3> &TurnRate,
                                       const double DeltaTime)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRefConst<T,3> Point2(Point2Ptr);

        /** rotation from ECEF frame to a local frame and add yaw */
        const VectorT<T, 3> ZAxis = VectorT<T,3>::UnitZ();
        const QuaternionT<T> QuatWorldToBodyToOdom = QuaternionT<T>::FromTwoVectors(ZAxis, Point1) * AngleAxisT<T>(*Yaw1Ptr, VectorT<T,3>::UnitZ());

        /** estimate translation */
        Velocity =  QuatWorldToBodyToOdom.conjugate() * (Point2 - Point1) / DeltaTime;

        /** estimate rotation */
        TurnRate.setZero();
        TurnRate(2) = NormalizeAngle<T>(*Yaw2Ptr - *Yaw1Ptr)/DeltaTime;
      }
  };
}

#endif // ODOMETRYMODEL_H
