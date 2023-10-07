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
* @file Example_AutoDiff.cpp
* @author Tim Pfeifer
* @date 19.03.2020
* @brief An example application to demonstrate how automatic differentiation can be used with ceres.
* @copyright GNU Public License.
*
*/

#include <ceres/ceres.h>
#include "libRSF.h"

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** create data objects */
  using JetType = ceres::Jet<double, 6 + 7>;

  libRSF::VectorT<JetType,3> Point1;
  libRSF::VectorT<JetType,3> Point2;

  libRSF::QuaternionT<JetType> Quat1;
  libRSF::QuaternionT<JetType> Quat2;

  libRSF::VectorT<JetType,6> Odom;
  libRSF::Vector6 OdomDouble;
  OdomDouble << 1,0,0,0,0,M_PI*2;

  /** init with data */
  const double DeltaTime = 0.1;

  Point1.setZero();
  Point1(0).v(0) = 1;
  Point1(1).v(1) = 1;
  Point1(2).v(2) = 1;

  Quat1.setIdentity();
  Quat1.x().v(3) = 1;
  Quat1.y().v(4) = 1;
  Quat1.z().v(5) = 1;
  Quat1.w().v(6) = 1;

  Odom = OdomDouble.template cast<JetType>();
  Odom(0).v(7) = 1;
  Odom(1).v(8) = 1;
  Odom(2).v(9) = 1;
  Odom(3).v(10) = 1;
  Odom(4).v(11) = 1;
  Odom(5).v(12) = 1;

  /** transform mean */
  libRSF::OdometryModel6DOF<JetType>::applyForward(Point1.data(), Quat1.coeffs().data(),
                                                   Point2.data(), Quat2.coeffs().data(),
                                                   Odom.head(3), Odom.tail(3), DeltaTime);

  libRSF::Quaternion Q2;
  Q2.x() = Quat2.x().a;
  Q2.y() = Quat2.y().a;
  Q2.z() = Quat2.z().a;
  Q2.w() = Quat2.w().a;

  libRSF::Vector3 P2;
  P2(0) = Point2(0).a;
  P2(1) = Point2(1).a;
  P2(2) = Point2(2).a;

  /** extract jacobian */
  libRSF::MatrixStatic<7,13> Jacobian;

  /** position jacobian */
  for(int n = 0; n < 7; n++)
  {
    if (n < 3)
    {
      Jacobian.row(n) = Point2(n).v;
    }
    else
    {
      Jacobian.row(n) = Quat2.coeffs()(n-3).v;
    }
  }

  /** transform jacobian to manifold*/
  libRSF::MatrixStatic<6,7> JacobianManifold;
  JacobianManifold.setIdentity();

  libRSF::Matrix34 JacobianQuat;
  libRSF::QuaternionError(Q2, libRSF::Quaternion::Identity(), &JacobianQuat, nullptr);
  JacobianManifold.block<3,4>(3,3) = JacobianQuat;

  /** compare against integrator */
  libRSF::OdometryIntegrator OdomInt;
  OdomInt.addMeasurement(OdomDouble.head(3), OdomDouble.tail(3), libRSF::Vector3::Ones()*0.1, libRSF::Vector3::Ones()*0.01, DeltaTime);


  /** print result */
  std::cout <<Jacobian << std::endl << std::endl;

  std::cout <<JacobianManifold*Jacobian << std::endl;

  return 0;
}
