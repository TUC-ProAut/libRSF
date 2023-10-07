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

#include "Geometry.h"

namespace libRSF
{
  /** angular conversion */
  double deg2rad(const double Deg)
  {
    return Deg / 180.0 * M_PI;
  }
  double rad2deg(const double Rad)
  {
    return Rad * 180.0 / M_PI;
  }

  Vector3 QuaternionError(const Quaternion& Q1,
                          const Quaternion& Q2,
                          Matrix34 *Jacobian1,
                          Matrix34 *Jacobian2)
  {
    if(Jacobian1 != nullptr && Jacobian2 != nullptr)
    {
      /** we want both jacobians */
      using JetType = ceres::Jet<double, 8>;

      /** init jets */
      QuaternionT<JetType> Q1Jet = Q1.template cast<JetType>();
      Q1Jet.x().v(0) = 1;
      Q1Jet.y().v(1) = 1;
      Q1Jet.z().v(2) = 1;
      Q1Jet.w().v(3) = 1;
      QuaternionT<JetType> Q2Jet = Q2.template cast<JetType>();
      Q1Jet.x().v(4) = 1;
      Q1Jet.y().v(5) = 1;
      Q1Jet.z().v(6) = 1;
      Q1Jet.w().v(7) = 1;

      /** apply transformation */
      const VectorT<JetType,3> ErrorJet = QuaternionError<JetType>(Q1Jet, Q2Jet);

      /** extract Jacobians */
      Jacobian1->row(0) = ErrorJet(0).v.head(4);
      Jacobian1->row(1) = ErrorJet(1).v.head(4);
      Jacobian1->row(2) = ErrorJet(2).v.head(4);

      Jacobian2->row(0) = ErrorJet(0).v.tail(4);
      Jacobian2->row(1) = ErrorJet(1).v.tail(4);
      Jacobian2->row(2) = ErrorJet(2).v.tail(4);

      /** extract mean */
      Vector3 Error;
      Error << ErrorJet(0).a, ErrorJet(1).a, ErrorJet(2).a;
      return Error;
    }
    else if(Jacobian1 != nullptr)
    {
      /** we want just jacobian 1 */
      typedef ceres::Jet<double, 4> JetType;

      /** init jets */
      QuaternionT<JetType> Q1Jet = Q1.template cast<JetType>();
      Q1Jet.x().v(0) = 1;
      Q1Jet.y().v(1) = 1;
      Q1Jet.z().v(2) = 1;
      Q1Jet.w().v(3) = 1;

      /** apply transformation */
      const VectorT<JetType,3> ErrorJet = QuaternionError<JetType>(Q1Jet, Q2.template cast<JetType>());

      /** extract jacobians */
      Jacobian1->row(0) = ErrorJet(0).v;
      Jacobian1->row(1) = ErrorJet(1).v;
      Jacobian1->row(2) = ErrorJet(2).v;

      /** extract mean */
      Vector3 Error;
      Error << ErrorJet(0).a, ErrorJet(1).a, ErrorJet(2).a;
      return Error;

    }
    else if(Jacobian2 != nullptr)
    {
      /** we want just jacobian 2 */
      typedef ceres::Jet<double, 4> JetType;

      /** init jets */
      QuaternionT<JetType> Q2Jet = Q2.template cast<JetType>();
      Q2Jet.x().v(0) = 1;
      Q2Jet.y().v(1) = 1;
      Q2Jet.z().v(2) = 1;
      Q2Jet.w().v(3) = 1;

      /** apply transformation */
      const VectorT<JetType,3> ErrorJet = QuaternionError<JetType>(Q1.template cast<JetType>(), Q2Jet);

      /** extract jacobians */
      Jacobian2->row(0) = ErrorJet(0).v;
      Jacobian2->row(1) = ErrorJet(1).v;
      Jacobian2->row(2) = ErrorJet(2).v;

      /** extract mean */
      Vector3 Error;
      Error << ErrorJet(0).a, ErrorJet(1).a, ErrorJet(2).a;
      return Error;
    }

    /** we want no jacobian */
    return QuaternionError<double>(Q1, Q2);
  }
}
