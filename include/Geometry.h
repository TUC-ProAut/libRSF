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
 * @file Geometry.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Simple geometric helper functions.
 * @copyright GNU Public License.
 *
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "NormalizeAngle.h"
#include "VectorMath.h"

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace libRSF
{
  /** angular conversion */
  double deg2rad(double Deg);
  double rad2deg(double Rad);

  /** 2D */
  template <typename T>
  MatrixT<T, 2, 2> RotationMatrix2D(T Yaw)
  {
    const Rotation2DT<T> Rot2D(Yaw);
    return Rot2D.toRotationMatrix();
  }

  template <typename T>
  VectorT<T, 3> RelativeMotion2D(const T* PointOld, const T* YawOld, const T* PointNew, const T* const YawNew)
  {
    const VectorT<T, 2> POld(PointOld[0], PointOld[1]);
    const VectorT<T, 2> PNew(PointNew[0], PointNew[1]);

    VectorT<T, 3> RelativeMotion;

    RelativeMotion.head(2) = RotationMatrix2D(YawOld[0]).transpose() * (PNew - POld);
    RelativeMotion(2) = NormalizeAngle(YawNew[0] - YawOld[0]);

    return RelativeMotion;
  }

  /** 3D */
  template<typename T>
  MatrixT<T,3,3> SkewSymmetricMatrix (const VectorT<T, 3> &Omega)
  {
    MatrixT<T,3,3> Mat;
    Mat <<  T(0.0),          -Omega.z(),    Omega.y(),
            Omega.z(),           T(0.0),   -Omega.x(),
           -Omega.y(),        Omega.x(),       T(0.0);

    return Mat;
  }

  template<typename T>
  QuaternionT<T> QuaternionExpMap (const VectorT<T, 3> &Angle)
  {
    /** convert to quaternion using axis-angle definition */
    QuaternionT<T> Quat;
    const T Norm = Angle.norm();
    if (Norm > T(1e-10))
    {
      Quat = AngleAxisT<T>(Norm, Angle/Norm);
    }
    else
    {
      /** use linear approximation for small angles (for stable derivative) */
      Quat.vec() = Angle/2.0;
      Quat.w() = T(1.0);
    }

    return Quat;
  }

  template<typename T>
  VectorT<T, 3> QuaternionLogMap (const QuaternionT<T> &Quat)
  {
    const AngleAxisT<T> AARot(Quat);

    VectorT<T, 3> Angle;
    if (abs(AARot.angle()) > T(1e-10))
    {
      Angle = AARot.axis()*AARot.angle();
    }
    else
    {
      /** use linear approximation for small angles (for stable derivative) */
      Angle = Quat.vec()*2.0;
    }

    return Angle;
  }

  template<typename T>
  QuaternionT<T> AngularVelocityToQuaternion (const VectorT<T, 3> &Omega, const double DeltaTime)
  {
    return QuaternionExpMap<T>(Omega*DeltaTime);
  }

  template<typename T>
  VectorT<T, 3> QuaternionToAngularVelocity (const QuaternionT<T> &Quat, const double DeltaTime)
  {
    return QuaternionLogMap<T>(Quat)/DeltaTime;
  }

  /** \brief Represents the operation Q1 - Q2 on a manifold
   *
   * \param Q1 first quaternion
   * \param Q2 second quaternion
   * \return 3x1 vector of the manifold error (related to an angular velocity)
   *
   */
  template <typename T>
  inline VectorT<T, 3> QuaternionError(const QuaternionT<T> Q1,
                                       const QuaternionT<T> Q2)
  {
    return QuaternionLogMap<T>(Q1 * Q2.conjugate());
  }

  /** \brief Represents the operation Q12 - (Q2 - Q1) on a manifold
   *
   * \param Q1 first quaternion
   * \param Q2 second quaternion
   * \param Q12 relative quaternion between Q1 and Q2
   * \return 3x1 vector of the manifold error (related to an angular velocity)
   *
   */
  template <typename T>
  inline VectorT<T, 3> RelativeQuaternionError(const QuaternionT<T> Quat1,
                                               const QuaternionT<T> Quat2,
                                               const QuaternionT<T> Quat12)
  {
    /** E_Q = (Q_12 * (Q_1^{-1} * Q_2)^{-1})[1:3] * 2 */
    return QuaternionError<T>(Quat12, Quat1.conjugate() * Quat2);
  }

  /** \brief Represents the operation Q1 - Q2 on a manifold with jacobians
   *
   * \param Q1 first quaternion
   * \param Q2 second quaternion
   * \param Jacobian1 derivative related to Q1
   * \param Jacobian2 derivative related to Q2
   * \return 3x1 vector of the manifold error (related to an angular velocity)
   *
   */
  Vector3 QuaternionError(const Quaternion& Q1,
                          const Quaternion& Q2,
                          Matrix34 *Jacobian1 = nullptr,
                          Matrix34 *Jacobian2 = nullptr);


  template<typename T>
  QuaternionT<T> RPYToQuaternion (const T Roll, const T Pitch, const T Yaw)
  {
    QuaternionT<T> Quat = AngleAxisT<T>(Roll, VectorT<T, 3>::UnitX())
                          * AngleAxisT<T>(Pitch, VectorT<T, 3>::UnitY())
                          * AngleAxisT<T>(Yaw, VectorT<T, 3>::UnitZ());
    return Quat;
  }

  /** \brief Converts a quaternion to Tait–Bryan angles
   *
   * \param Quat eigen quaternion with ordering x,y,z,w
   * \return [Roll, Pitch, Yaw]
   *
   */
  template<typename T>
  VectorT<T, 3> QuaternionToRPYEigen(QuaternionT<T> Quat)
  {
    VectorT<T, 3> RPY = Quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    /** catch pi overflow */
    VectorT<T, 3> RPY2;
    if(RPY(2) > T(M_PI_2))
    {
      RPY2(2) = RPY(2) - M_PI;

      if(RPY(1) > T(M_PI_2))
      {
        RPY2(1) = - RPY(1) + M_PI;

        if(RPY(0) > T(M_PI_2))
        {
          RPY2(0) = RPY(0) - M_PI;
        }
        else if(RPY(0) < T(-M_PI_2))
        {
          RPY2(0) = RPY(0) + M_PI;
        }
      }
      else if(RPY(1) < T(-M_PI_2))
      {
        RPY2(1) = -M_PI - RPY(1);

        if(RPY(0) > T(M_PI_2))
        {
          RPY2(0) = RPY(0) - M_PI;
        }
        else if(RPY(0) < T(-M_PI_2))
        {
          RPY2(0) = RPY(0) + M_PI;
        }
      }
    }
    else
    {
      RPY2 = RPY;
    }

    return RPY2;
  }

  template<typename T>
  VectorT<T, 3> QuaternionToRPY(QuaternionT<T> Q)
  {
    VectorT<T, 3> RPY;

    T Delta = Q.w()*Q.y() - Q.x()*Q.z();
    if(Delta == T(-1/2))
    {
      RPY(0) = 2.0 * atan2(Q.x(), Q.w());
      RPY(1) = T(-M_PI_2);
      RPY(2) = T(0);
    }
    else if(Delta == T(1/2))
    {
      RPY(0) = -2.0 * atan2(Q.x(), Q.w());
      RPY(1) = T(-M_PI_2);
      RPY(2) = T(0);
    }
    else
    {
      RPY(0) = atan2(2.0 * (Q.w()*Q.z() + Q.x()*Q.y()), 1.0 - 2.0 * (pow(Q.y(),2) + pow(Q.z(),2)));
      RPY(1) = asin(2.0 * Delta);
      RPY(2) = atan2(2.0 * (Q.w()*Q.x() + Q.y()*Q.z()), 1.0 - 2.0 * (pow(Q.x(),2) + pow(Q.y(),2)));
    }

    return RPY.reverse();
  }

  template <typename T>
  T HeightToPressure(const T Height)
  {
    return 101325.0 * pow(1.0 - 0.0065 * Height / 288.15, 5.255);
  }
}

#endif // GEOMETRY_H
