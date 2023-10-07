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
 * @file RelativePoseModel.h
 * @author Tim Pfeifer
 * @date 20.05.2020
 * @brief Implementation of geometric models to describe relative poses.
 * @copyright GNU Public License.
 *
 */

#ifndef RELATIVEPOSEMODEL_H
#define RELATIVEPOSEMODEL_H

#include "../Geometry.h"

namespace libRSF
{
  template<typename T>
  class RelativePose2Model
  {
    public:
      RelativePose2Model() = default;
      virtual ~RelativePose2Model() = default;

      void static inline applyForward(const T* Point1Ptr,
                                      const T* Yaw1Ptr,
                                      T* Point2Ptr,
                                      T* Yaw2Ptr,
                                      const VectorT<T, 2> Trans12,
                                      const Rotation2DT<T> Rot12)
      {
        /** wrap as eigen types */
        VectorRefConst<T,2> Point1(Point1Ptr);
        VectorRef<T,2>      Point2(Point2Ptr);
        const Rotation2DT<T> Rot1(Yaw1Ptr[0]);

        /** apply transformation */
        Point2 = Point1 + Rot1 * Trans12;
        Yaw2Ptr[0] = (Rot1*Rot12).smallestAngle();
      }

      void static inline applyBackward(const T* Point1Ptr,
                                       const T* Yaw1Ptr,
                                       const T* Point2Ptr,
                                       const T* Yaw2Ptr,
                                       VectorT<T, 2> &Trans12,
                                       Rotation2DT<T> &Rot12)
      {
        /** wrap as eigen types */
        VectorRefConst<T,2> Point1(Point1Ptr);
        VectorRefConst<T,2> Point2(Point2Ptr);
        const Rotation2DT<T> Rot1(Yaw1Ptr[0]);
        const Rotation2DT<T> Rot2(Yaw2Ptr[0]);

        /** transform backward */
        Trans12 = Rot1.inverse() * (Point2 - Point1);
        Rot12 = Rot1.inverse() * Rot2;
      }
  };

  template<typename T>
  class RelativePose3Model
  {
    public:
      RelativePose3Model() = default;
      virtual ~RelativePose3Model() = default;

      void static inline applyForward(const T* Point1Ptr,
                                      const T* Quat1Ptr,
                                      T* Point2Ptr,
                                      T* Quat2Ptr,
                                      const VectorT<T, 3> Trans12,
                                      const QuaternionT<T> Quat12)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRef<T,3>      Point2(Point2Ptr);
        QuaternionRefConst<T> Quat1(Quat1Ptr);
        QuaternionRef<T>      Quat2(Quat2Ptr);

        /** apply transformation */
        Point2 = Point1 + Quat1 * Trans12;
        Quat2 = (Quat1 * Quat12).normalized();
      }

      void static inline applyBackward(const T* Point1Ptr,
                                       const T* Quat1Ptr,
                                       const T* Point2Ptr,
                                       const T* Quat2Ptr,
                                       VectorT<T, 3> &Trans12,
                                       QuaternionT<T> &Quat12)
      {
        /** wrap as eigen types */
        VectorRefConst<T,3> Point1(Point1Ptr);
        VectorRefConst<T,3> Point2(Point2Ptr);
        QuaternionRefConst<T> Quat1(Quat1Ptr);
        QuaternionRefConst<T> Quat2(Quat2Ptr);

        /** transform backward */
        Trans12 = Quat1.conjugate() * (Point2 - Point1);
        Quat12 = Quat1.conjugate() * Quat2;
      }
  };
}

#endif // RELATIVEPOSEMODEL_H
