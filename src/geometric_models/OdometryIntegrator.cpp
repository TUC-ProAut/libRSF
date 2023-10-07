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

#include "geometric_models/OdometryIntegrator.h"

namespace libRSF
{
  OdometryIntegrator::OdometryIntegrator()
  {
    this->reset();
  }

  void OdometryIntegrator::reset()
  {
    this->Time_ = 0;

    this->Translation_.setZero();
    this->Rotation_.setIdentity();

    /** small initial Cov to avoid rank deficiency*/
    this->PoseCov_ = Matrix77::Identity() * 1e-8;
  }

  double OdometryIntegrator::getTime() const
  {
    return this->Time_;
  }

  void OdometryIntegrator::getPose(Vector3 &Translation, Quaternion &Rotation) const
  {
    Translation = this->Translation_;
    Rotation = this->Rotation_;
  }

  void OdometryIntegrator::getCov(Matrix33 &TranslationCov, Matrix44 &RotationCov) const
  {
    TranslationCov = this->PoseCov_.topLeftCorner<3,3>();
    RotationCov = this->PoseCov_.bottomRightCorner<4,4>();
  }

  Vector7 OdometryIntegrator::getJointPose() const
  {
    return (Vector7() << this->Translation_, this->Rotation_.coeffs()).finished();
  }

  Matrix66 OdometryIntegrator::getJointCovOnManifold() const
  {
    Matrix66 Cov = Matrix66::Zero();

    Matrix33 CovTrans, CovRot;
    this->getCovOnManifold(CovTrans, CovRot);

    Cov.block<3,3>(0,0) = CovTrans;
    Cov.block<3,3>(3,3) = CovRot;

    return Cov;
  }

  void OdometryIntegrator::getCovOnManifold(Matrix33 &TranslationCov, Matrix33 &RotationCov) const
  {
    /** translation stays the same */
    TranslationCov = this->PoseCov_.topLeftCorner<3,3>();

    /** rotation has to be projected on the manifold */
    Matrix34 JacQuat;
    QuaternionError(this->Rotation_, this->Rotation_, &JacQuat, nullptr);
    RotationCov = JacQuat * this->PoseCov_.bottomRightCorner<4,4>() * JacQuat.transpose();
  }

  void OdometryIntegrator::addMeasurement(const libRSF::Data &Odom, const double DeltaTime)
  {
    this->addMeasurement(Odom.getMean().head(3), Odom.getMean().tail(3),
                         Odom.getCovarianceDiagonal().head(3), Odom.getCovarianceDiagonal().tail(3),
                         DeltaTime);
  }

  void OdometryIntegrator::addMeasurement(const Vector3 &Velocity, const Vector3 &TurnRate,
                                              const Vector3 &VelocityCov, const Vector3 &TurnRateCov,
                                              const double DeltaTime)
  {
    /** define basic autodiff type [Quat1, Vel, TR] */
    using JetType = ceres::Jet<double, 13>;

    /** old states */
    VectorT<JetType,3> Point1Jet = this->Translation_.template cast<JetType>();
    Point1Jet.x().v(0) = 1;
    Point1Jet.y().v(1) = 1;
    Point1Jet.z().v(2) = 1;

    QuaternionT<JetType> Quat1Jet = this->Rotation_.template cast<JetType>();
    Quat1Jet.x().v(3) = 1;
    Quat1Jet.y().v(4) = 1;
    Quat1Jet.z().v(5) = 1;
    Quat1Jet.w().v(6) = 1;

    /** measurements */
    VectorT<JetType,3> VelocityJet = Velocity.template cast<JetType>();
    VelocityJet.x().v(7) = 1;
    VelocityJet.y().v(8) = 1;
    VelocityJet.z().v(9) = 1;

    VectorT<JetType,3> TurnRateJet = TurnRate.template cast<JetType>();
    TurnRateJet.x().v(10) = 1;
    TurnRateJet.y().v(11) = 1;
    TurnRateJet.z().v(12) = 1;

    /** new result states */
    VectorT<JetType,3> Point2Jet;
    QuaternionT<JetType> Quat2Jet;

    /** perform integration */
    OdometryModel6DOF<JetType>::applyForward(Point1Jet.data(), Quat1Jet.coeffs().data(),
                                             Point2Jet.data(), Quat2Jet.coeffs().data(),
                                             VelocityJet, TurnRateJet, DeltaTime);

    /** store mean */
    this->Translation_.x() = Point2Jet.x().a;
    this->Translation_.y() = Point2Jet.y().a;
    this->Translation_.z() = Point2Jet.z().a;

    this->Rotation_.x() = Quat2Jet.x().a;
    this->Rotation_.y() = Quat2Jet.y().a;
    this->Rotation_.z() = Quat2Jet.z().a;
    this->Rotation_.w() = Quat2Jet.w().a;

    /** extract jacobian */
    MatrixStatic<7,13> Jacobian;

    Jacobian.row(0) = Point2Jet.x().v;
    Jacobian.row(1) = Point2Jet.y().v;
    Jacobian.row(2) = Point2Jet.z().v;

    Jacobian.row(3) = Quat2Jet.x().v;
    Jacobian.row(4) = Quat2Jet.y().v;
    Jacobian.row(5) = Quat2Jet.z().v;
    Jacobian.row(6) = Quat2Jet.w().v;

    /** build full covariance */
    MatrixStatic<13,13> Covariance;
    Covariance.setZero();
    Covariance.topLeftCorner<7,7>() = this->PoseCov_;
    Covariance.bottomRightCorner<6,6>().topLeftCorner<3,3>() = VelocityCov.asDiagonal();
    Covariance.bottomRightCorner<3,3>() = TurnRateCov.asDiagonal();

    /** propagate covariance */
    this->PoseCov_ = Jacobian*Covariance*Jacobian.transpose();
    this->Time_ += DeltaTime;
  }
}
