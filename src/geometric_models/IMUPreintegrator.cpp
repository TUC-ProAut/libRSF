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

#include "geometric_models/IMUPreintegrator.h"

namespace libRSF
{
  IMUPreintegrator::IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                                     const double RandomWalkAcc, const double RandomWalkGyro,
                                     const double CurrentTime): CurrentTime_(CurrentTime),
      NoiseDensityAcc_(0.0),
      NoiseDensityGyro_(0.0),
      RandomWalkAcc_(RandomWalkAcc),
      RandomWalkGyro_(RandomWalkGyro),
      BiasAcc_(BiasAcc),
      BiasTR_(BiasTR)
  {
    initialize_();
  }

  IMUPreintegrator::IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                                     const double NoiseDensityAcc, const double NoiseDensityGyro,
                                     const double RandomWalkAcc, const double RandomWalkGyro,
                                     const double CurrentTime): CurrentTime_(CurrentTime),
        NoiseDensityAcc_(NoiseDensityAcc),
        NoiseDensityGyro_(NoiseDensityGyro),
        RandomWalkAcc_(RandomWalkAcc),
        RandomWalkGyro_(RandomWalkGyro),
        BiasAcc_(BiasAcc),
        BiasTR_(BiasTR)
  {
    initialize_();
  }

  void IMUPreintegrator::initialize_()
  {
    /** mean */
    Translation_ = Vector3::Zero();
    Velocity_ = Vector3::Zero();
    Rotation_ = Quaternion::Identity();

    /** jacobians */
    JacTransBiasAcc_.setZero();
    JacTransBiasTR_.setZero();
    JacVelBiasAcc_.setZero();
    JacVelBiasTR_.setZero();
    JacRotBiasTR_.setZero();

    /** small initial Cov to avoid rank deficiency*/
    TVRCov_ = Matrix1010::Identity() * 1e-8;

    /** length of integration */
    DeltaTime_ = 0;
  }

  void IMUPreintegrator::updateBias(const Vector3 &BiasAcc, const Vector3 &BiasTR)
  {
    const double StartTime = CurrentTime_ - DeltaTime_;
    this->initialize_();

    /** initial values */
    CurrentTime_ = StartTime;
    BiasAcc_ = BiasAcc;
    BiasTR_ = BiasTR;

    /** perform full pre-integration */
    integrateFull_(CurrentTime_);
  }

  PreintegratedIMUResult IMUPreintegrator::getPreintegratedState()
  {
    PreintegratedIMUResult Result;

    /** measurement biases */
    Result.BiasAcc = BiasAcc_;
    Result.BiasTR = BiasTR_;

    /** pre-integration objects*/
    Result.Translation = Translation_;
    Result.Velocity = Velocity_;
    Result.Rotation = Rotation_;

    /** calculate rotation manifold jacobian */
    Matrix34 JacQuat;
    QuaternionError(Rotation_, Quaternion::Identity(), &JacQuat, nullptr);
    MatrixStatic<9,10> JacFull = MatrixStatic<9,10>::Zero();
    JacFull.topLeftCorner<9,9>().setIdentity();
    JacFull.bottomRightCorner<3,4>() = JacQuat;

    /** covariance matrix */
    Result.PreIntCov.setZero();
    Result.PreIntCov.topLeftCorner<9,9>() = JacFull * TVRCov_ * JacFull.transpose();
    Result.PreIntCov.bottomRightCorner<6,6>().topLeftCorner<3,3>() = Matrix33::Identity() * RandomWalkAcc_ * RandomWalkAcc_ * DeltaTime_;
    Result.PreIntCov.bottomRightCorner<3,3>() = Matrix33::Identity() * RandomWalkGyro_ * RandomWalkGyro_ * DeltaTime_;

    /** Jacobians of the pre-integrated objects w.r.t. the measurement biases*/
    Result.JacTransBiasAcc = JacTransBiasAcc_;
    Result.JacTransBiasTR = JacTransBiasTR_;

    Result.JacVelBiasAcc = JacVelBiasAcc_;
    Result.JacVelBiasTR = JacVelBiasTR_;

    /** Jacobians in local tangent space */
    Result.JacRotBiasTRLocal = JacQuat * JacRotBiasTR_;

    /** length of the integral in time */
    Result.DeltaTime = DeltaTime_;

    /** calculate start time */
    Result.StartTime = CurrentTime_ - DeltaTime_;

    /** measurements */
    Result.Measurements = Measurements_;

    return Result;
  }

  void IMUPreintegrator::addMeasurement(const Data &IMUMeasurement)
  {
    /** add to measurement list and perform an integration step */
    Measurements_.emplace_back(IMUMeasurement);
    this->integrateSingleMeasurement_(Measurements_.size()-1,IMUMeasurement.getTimestamp());
  }

  void IMUPreintegrator::integrateToTime(const double Timestamp)
  {
    this->integrateSingleMeasurement_(Measurements_.size()-1,Timestamp);
  }

  void IMUPreintegrator::integrateFull_(const double Timestamp)
  {
    /** loop over measurements and integrate */
    for(int i = 0; i < static_cast<int>(Measurements_.size()); i++)
    {
      this->integrateSingleMeasurement_(i, Measurements_.at(i).getTimestamp());
    }
    this->integrateSingleMeasurement_(Measurements_.size()-1,Timestamp);
  }

  void IMUPreintegrator::integrateSingleMeasurement_(const int Index, const double Timestamp)
  {
    /** calculate delta t */
    const double dt = Timestamp - CurrentTime_;

    /** get measurements*/
    Vector3 Acceleration2 = Measurements_.at(Index).getMean().head(3);
    Vector3 TurnRate = Measurements_.at(Index).getMean().tail(3);

    /** average for mid-point integration */
    Vector3 Acceleration1;
    if (Index > 0)
    {
      Acceleration1 = Measurements_.at(Index-1).getMean().head(3);
      TurnRate = 0.5*(TurnRate + Measurements_.at(Index-1).getMean().tail(3));
    }
    else
    {
      Acceleration1 = Acceleration2;
    }

    /** get measurement covariances */
    Matrix33 AccelerationCov;
    if(NoiseDensityAcc_ <= 0.0)
    {
      AccelerationCov =
          Measurements_.at(Index).getCovarianceDiagonal().head(3).asDiagonal();
    }
    else
    {
      AccelerationCov = Matrix33::Identity() * NoiseDensityAcc_ * NoiseDensityAcc_ / dt;
    }

    Matrix33 TurnRateCov ;
    if(NoiseDensityGyro_ <= 0.0)
    {
      TurnRateCov =
          Measurements_.at(Index).getCovarianceDiagonal().tail(3).asDiagonal();
    }
    else
    {
      TurnRateCov = Matrix33::Identity() * NoiseDensityGyro_ * NoiseDensityGyro_ / dt;
    }

    /** autodiff integration */
    using JetType = ceres::Jet<double, 22>;

    /** set up old states */
    VectorT<JetType,3> Translation1Jet = this->Translation_.template cast<JetType>();
    Translation1Jet.x().v(0) = 1;
    Translation1Jet.y().v(1) = 1;
    Translation1Jet.z().v(2) = 1;
    Translation1Jet.x().v.template segment<3>(10) = JacTransBiasAcc_.row(0);
    Translation1Jet.y().v.template segment<3>(10) = JacTransBiasAcc_.row(1);
    Translation1Jet.z().v.template segment<3>(10) = JacTransBiasAcc_.row(2);
    Translation1Jet.x().v.template segment<3>(13) = JacTransBiasTR_.row(0);
    Translation1Jet.y().v.template segment<3>(13) = JacTransBiasTR_.row(1);
    Translation1Jet.z().v.template segment<3>(13) = JacTransBiasTR_.row(2);

    VectorT<JetType,3> Velocity1Jet = this->Velocity_.template cast<JetType>();
    Velocity1Jet.x().v(3) = 1;
    Velocity1Jet.y().v(4) = 1;
    Velocity1Jet.z().v(5) = 1;
    Velocity1Jet.x().v.template segment<3>(10) = JacVelBiasAcc_.row(0);
    Velocity1Jet.y().v.template segment<3>(10) = JacVelBiasAcc_.row(1);
    Velocity1Jet.z().v.template segment<3>(10) = JacVelBiasAcc_.row(2);
    Velocity1Jet.x().v.template segment<3>(13) = JacVelBiasTR_.row(0);
    Velocity1Jet.y().v.template segment<3>(13) = JacVelBiasTR_.row(1);
    Velocity1Jet.z().v.template segment<3>(13) = JacVelBiasTR_.row(2);

    QuaternionT<JetType> Rotation1Jet = this->Rotation_.template cast<JetType>();
    Rotation1Jet.x().v(6) = 1;
    Rotation1Jet.y().v(7) = 1;
    Rotation1Jet.z().v(8) = 1;
    Rotation1Jet.w().v(9) = 1;
    Rotation1Jet.x().v.template segment<3>(13) = JacRotBiasTR_.row(0);
    Rotation1Jet.y().v.template segment<3>(13) = JacRotBiasTR_.row(1);
    Rotation1Jet.z().v.template segment<3>(13) = JacRotBiasTR_.row(2);
    Rotation1Jet.w().v.template segment<3>(13) = JacRotBiasTR_.row(3);

    /** biases */
    VectorT<JetType,3> BiasAccJet = this->BiasAcc_.template cast<JetType>();
    BiasAccJet.x().v(10) = 1;
    BiasAccJet.y().v(11) = 1;
    BiasAccJet.z().v(12) = 1;

    VectorT<JetType,3> BiasTRJet = this->BiasTR_.template cast<JetType>();
    BiasTRJet.x().v(13) = 1;
    BiasTRJet.y().v(14) = 1;
    BiasTRJet.z().v(15) = 1;

    /** measurements */
    VectorT<JetType,3> Acceleration1Jet = Acceleration1.template cast<JetType>();
    Acceleration1Jet.x().v(16) = 1;
    Acceleration1Jet.y().v(17) = 1;
    Acceleration1Jet.z().v(18) = 1;

    VectorT<JetType,3> Acceleration2Jet = Acceleration2.template cast<JetType>();
    Acceleration2Jet.x().v(16) = 1;
    Acceleration2Jet.y().v(17) = 1;
    Acceleration2Jet.z().v(18) = 1;

    VectorT<JetType,3> TurnRateJet = TurnRate.template cast<JetType>();
    TurnRateJet.x().v(19) = 1;
    TurnRateJet.y().v(20) = 1;
    TurnRateJet.z().v(21) = 1;

    /** calculate integration */
    const QuaternionT<JetType> Rotation2Jet = Rotation1Jet * AngularVelocityToQuaternion<JetType>(TurnRateJet - BiasTRJet, dt);

    const VectorT<JetType,3> AccMidPoint = 0.5 * (Rotation1Jet*(Acceleration1Jet - BiasAccJet) + Rotation2Jet*(Acceleration2Jet - BiasAccJet));

    const VectorT<JetType,3> Translation2Jet = Translation1Jet + dt*Velocity1Jet + 0.5*dt*dt*AccMidPoint;

    const VectorT<JetType,3> Velocity2Jet = Velocity1Jet + dt*AccMidPoint;

    /** extract mean */
    this->Translation_.x() = Translation2Jet.x().a;
    this->Translation_.y() = Translation2Jet.y().a;
    this->Translation_.z() = Translation2Jet.z().a;

    this->Velocity_.x() = Velocity2Jet.x().a;
    this->Velocity_.y() = Velocity2Jet.y().a;
    this->Velocity_.z() = Velocity2Jet.z().a;

    this->Rotation_.x() = Rotation2Jet.x().a;
    this->Rotation_.y() = Rotation2Jet.y().a;
    this->Rotation_.z() = Rotation2Jet.z().a;
    this->Rotation_.w() = Rotation2Jet.w().a;

    /** extract Jacobian */
    MatrixStatic<10,16> Jacobian = MatrixStatic<10, 16>::Zero();
    Jacobian.row(0).head<10>() = Translation2Jet.x().v.template head<10>();
    Jacobian.row(1).head<10>() = Translation2Jet.y().v.template head<10>();
    Jacobian.row(2).head<10>() = Translation2Jet.z().v.template head<10>();
    Jacobian.row(0).tail<6>() = Translation2Jet.x().v.template tail<6>();
    Jacobian.row(1).tail<6>() = Translation2Jet.y().v.template tail<6>();
    Jacobian.row(2).tail<6>() = Translation2Jet.z().v.template tail<6>();

    Jacobian.row(3).head<10>() = Velocity2Jet.x().v.template head<10>();
    Jacobian.row(4).head<10>() = Velocity2Jet.y().v.template head<10>();
    Jacobian.row(5).head<10>() = Velocity2Jet.z().v.template head<10>();
    Jacobian.row(3).tail<6>() = Velocity2Jet.x().v.template tail<6>();
    Jacobian.row(4).tail<6>() = Velocity2Jet.y().v.template tail<6>();
    Jacobian.row(5).tail<6>() = Velocity2Jet.z().v.template tail<6>();

    Jacobian.row(6).head<10>() = Rotation2Jet.x().v.template head<10>();
    Jacobian.row(7).head<10>() = Rotation2Jet.y().v.template head<10>();
    Jacobian.row(8).head<10>() = Rotation2Jet.z().v.template head<10>();
    Jacobian.row(9).head<10>() = Rotation2Jet.w().v.template head<10>();
    Jacobian.row(6).tail<6>() = Rotation2Jet.x().v.template tail<6>();
    Jacobian.row(7).tail<6>() = Rotation2Jet.y().v.template tail<6>();
    Jacobian.row(8).tail<6>() = Rotation2Jet.z().v.template tail<6>();
    Jacobian.row(9).tail<6>() = Rotation2Jet.w().v.template tail<6>();

    /** set up input covariance (this is for euler integration) */
    MatrixStatic<16, 16> Covariance = MatrixStatic<16, 16>::Zero();
    Covariance.topLeftCorner<10,10>() = this->TVRCov_;
    Covariance.bottomRightCorner<6,6>().topLeftCorner<3,3>() = AccelerationCov;
    Covariance.bottomRightCorner<3,3>() = TurnRateCov;

    /** propagate covariance */
    this->TVRCov_ = Jacobian*Covariance*Jacobian.transpose();

    /** update bias jacobians */
    this->JacTransBiasAcc_.row(0) = Translation2Jet.x().v.template segment<3>(10);
    this->JacTransBiasAcc_.row(1) = Translation2Jet.y().v.template segment<3>(10);
    this->JacTransBiasAcc_.row(2) = Translation2Jet.z().v.template segment<3>(10);

    this->JacVelBiasAcc_.row(0) = Velocity2Jet.x().v.template segment<3>(10);
    this->JacVelBiasAcc_.row(1) = Velocity2Jet.y().v.template segment<3>(10);
    this->JacVelBiasAcc_.row(2) = Velocity2Jet.z().v.template segment<3>(10);

    this->JacTransBiasTR_.row(0) = Translation2Jet.x().v.template segment<3>(13);
    this->JacTransBiasTR_.row(1) = Translation2Jet.y().v.template segment<3>(13);
    this->JacTransBiasTR_.row(2) = Translation2Jet.z().v.template segment<3>(13);

    this->JacVelBiasTR_.row(0) = Velocity2Jet.x().v.template segment<3>(13);
    this->JacVelBiasTR_.row(1) = Velocity2Jet.y().v.template segment<3>(13);
    this->JacVelBiasTR_.row(2) = Velocity2Jet.z().v.template segment<3>(13);

    this->JacRotBiasTR_.row(0) = Rotation2Jet.x().v.template segment<3>(13);
    this->JacRotBiasTR_.row(1) = Rotation2Jet.y().v.template segment<3>(13);
    this->JacRotBiasTR_.row(2) = Rotation2Jet.z().v.template segment<3>(13);
    this->JacRotBiasTR_.row(3) = Rotation2Jet.w().v.template segment<3>(13);

    /** update time variables */
    CurrentTime_ = Timestamp;
    DeltaTime_ += dt;
  }
}
