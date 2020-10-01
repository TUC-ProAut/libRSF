/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
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
                                     const double CurrentTime): _CurrentTime(CurrentTime),
                                     _BiasAcc(BiasAcc), _BiasTR(BiasTR),
                                     _NoiseDensityAcc(0.0), _NoiseDensityGyro(0.0),
                                     _RandomWalkAcc(RandomWalkAcc), _RandomWalkGyro(RandomWalkGyro)
  {
    initialize();
  }

  IMUPreintegrator::IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                                     const double NoiseDensityAcc, const double NoiseDensityGyro,
                                     const double RandomWalkAcc, const double RandomWalkGyro,
                                     const double CurrentTime):_CurrentTime(CurrentTime),
                                     _BiasAcc(BiasAcc), _BiasTR(BiasTR),
                                     _NoiseDensityAcc(NoiseDensityAcc), _NoiseDensityGyro(NoiseDensityGyro),
                                     _RandomWalkAcc(RandomWalkAcc), _RandomWalkGyro(RandomWalkGyro)
  {
    initialize();
  }

  void IMUPreintegrator::initialize()
  {
    /** mean */
    _Translation = Vector3::Zero();
    _Velocity = Vector3::Zero();
    _Rotation = Quaternion::Identity();

    /** jacobians */
    _JacTransBiasAcc.setZero();
    _JacTransBiasTR.setZero();
    _JacVelBiasAcc.setZero();
    _JacVelBiasTR.setZero();
    _JacRotBiasTR.setZero();

    /** small initial Cov to avoid rank deficiency*/
    _TVRCov = Matrix1010::Identity() * 1e-8;

    /** length of integration */
    _DeltaTime = 0;
  }

  void IMUPreintegrator::updateBias(const Vector3 &BiasAcc, const Vector3 &BiasTR)
  {
    const double StartTime = _CurrentTime - _DeltaTime;
    this->initialize();

    /** initial values */
    _CurrentTime = StartTime;
    _BiasAcc = BiasAcc;
    _BiasTR = BiasTR;

    /** perform full pre-integration */
    integrateFull(_CurrentTime);
  }

  PreintegratedIMUResult IMUPreintegrator::getPreintegratedState()
  {
    PreintegratedIMUResult Result;

    /** measurement biases */
    Result.BiasAcc = _BiasAcc;
    Result.BiasTR = _BiasTR;

    /** pre-integration objects*/
    Result.Translation = _Translation;
    Result.Velocity = _Velocity;
    Result.Rotation = _Rotation;

    /** calculate rotation manifold jacobian */
    Matrix34 JacQuat;
    QuaternionError(_Rotation, Quaternion::Identity(), &JacQuat, nullptr);
    MatrixStatic<9,10> JacFull = MatrixStatic<9,10>::Zero();
    JacFull.topLeftCorner<9,9>().setIdentity();
    JacFull.bottomRightCorner<3,4>() = JacQuat;

    /** covariance matrix */
    Result.PreIntCov.setZero();
    Result.PreIntCov.topLeftCorner<9,9>() = JacFull * _TVRCov * JacFull.transpose();
    Result.PreIntCov.bottomRightCorner<6,6>().topLeftCorner<3,3>() = Matrix33::Identity() * _RandomWalkAcc * _RandomWalkAcc * _DeltaTime;
    Result.PreIntCov.bottomRightCorner<3,3>() = Matrix33::Identity() * _RandomWalkGyro * _RandomWalkGyro * _DeltaTime;

    /** Jacobians of the pre-integrated objects w.r.t. the measurement biases*/
    Result.JacTransBiasAcc = _JacTransBiasAcc;
    Result.JacTransBiasTR = _JacTransBiasTR;

    Result.JacVelBiasAcc = _JacVelBiasAcc;
    Result.JacVelBiasTR = _JacVelBiasTR;

    /** Jacobians in local tangent space */
    Result.JacRotBiasTRLocal = JacQuat * _JacRotBiasTR;

    /** length of the integral in time */
    Result.DeltaTime = _DeltaTime;

    /** calculate start time */
    Result.StartTime = _CurrentTime - _DeltaTime;

    /** measurements */
    Result.Measurements = _Measurements;

    return Result;
  }

  void IMUPreintegrator::addMeasurement(const SensorData &IMUMeasurement)
  {
    /** add to measurement list and perform an integration step */
    _Measurements.emplace_back(IMUMeasurement);
    this->integrateSingleMeasurement(_Measurements.size()-1,IMUMeasurement.getTimestamp());
  }

  void IMUPreintegrator::integrateToTime(const double Timestamp)
  {
    this->integrateSingleMeasurement(_Measurements.size()-1,Timestamp);
  }

  void IMUPreintegrator::integrateFull(const double Timestamp)
  {
    /** loop over measurements and integrate */
    for(int i = 0; i < _Measurements.size(); i++)
    {
      this->integrateSingleMeasurement(i, _Measurements.at(i).getTimestamp());
    }
    this->integrateSingleMeasurement(_Measurements.size()-1,Timestamp);
  }

  void IMUPreintegrator::integrateSingleMeasurement(const int Index, const double Timestamp)
  {
    /** calculate delta t */
    const double dt = Timestamp - _CurrentTime;

    /** get measurements*/
    Vector3 Acceleration2 = _Measurements.at(Index).getMean().head(3);
    Vector3 TurnRate = _Measurements.at(Index).getMean().tail(3);

    /** average for mid-point integration */
    Vector3 Acceleration1;
    if (Index > 0)
    {
      Acceleration1 = _Measurements.at(Index-1).getMean().head(3);
      TurnRate = 0.5*(TurnRate + _Measurements.at(Index-1).getMean().tail(3));
    }
    else
    {
      Acceleration1 = Acceleration2;
    }

    /** get measurement covariances */
    Matrix33 AccelerationCov;
    if(_NoiseDensityAcc <= 0.0)
    {
      AccelerationCov = _Measurements.at(Index).getCovariance().head(3).asDiagonal();
    }
    else
    {
      AccelerationCov = Matrix33::Identity() * _NoiseDensityAcc * _NoiseDensityAcc / dt;
    }

    Matrix33 TurnRateCov ;
    if(_NoiseDensityGyro <= 0.0)
    {
      TurnRateCov = _Measurements.at(Index).getCovariance().tail(3).asDiagonal();
    }
    else
    {
      TurnRateCov = Matrix33::Identity() * _NoiseDensityGyro * _NoiseDensityGyro / dt;
    }

    /** autodiff integration */
    typedef ceres::Jet<double, 22> JetType;

    /** set up old states */
    VectorT<JetType,3> Translation1Jet = this->_Translation.template cast<JetType>();
    Translation1Jet.x().v(0) = 1;
    Translation1Jet.y().v(1) = 1;
    Translation1Jet.z().v(2) = 1;
    Translation1Jet.x().v.template segment<3>(10) = _JacTransBiasAcc.row(0);
    Translation1Jet.y().v.template segment<3>(10) = _JacTransBiasAcc.row(1);
    Translation1Jet.z().v.template segment<3>(10) = _JacTransBiasAcc.row(2);
    Translation1Jet.x().v.template segment<3>(13) = _JacTransBiasTR.row(0);
    Translation1Jet.y().v.template segment<3>(13) = _JacTransBiasTR.row(1);
    Translation1Jet.z().v.template segment<3>(13) = _JacTransBiasTR.row(2);

    VectorT<JetType,3> Velocity1Jet = this->_Velocity.template cast<JetType>();
    Velocity1Jet.x().v(3) = 1;
    Velocity1Jet.y().v(4) = 1;
    Velocity1Jet.z().v(5) = 1;
    Velocity1Jet.x().v.template segment<3>(10) = _JacVelBiasAcc.row(0);
    Velocity1Jet.y().v.template segment<3>(10) = _JacVelBiasAcc.row(1);
    Velocity1Jet.z().v.template segment<3>(10) = _JacVelBiasAcc.row(2);
    Velocity1Jet.x().v.template segment<3>(13) = _JacVelBiasTR.row(0);
    Velocity1Jet.y().v.template segment<3>(13) = _JacVelBiasTR.row(1);
    Velocity1Jet.z().v.template segment<3>(13) = _JacVelBiasTR.row(2);

    QuaternionT<JetType> Rotation1Jet = this->_Rotation.template cast<JetType>();
    Rotation1Jet.x().v(6) = 1;
    Rotation1Jet.y().v(7) = 1;
    Rotation1Jet.z().v(8) = 1;
    Rotation1Jet.w().v(9) = 1;
    Rotation1Jet.x().v.template segment<3>(13) = _JacRotBiasTR.row(0);
    Rotation1Jet.y().v.template segment<3>(13) = _JacRotBiasTR.row(1);
    Rotation1Jet.z().v.template segment<3>(13) = _JacRotBiasTR.row(2);
    Rotation1Jet.w().v.template segment<3>(13) = _JacRotBiasTR.row(3);

    /** biases */
    VectorT<JetType,3> BiasAccJet = this->_BiasAcc.template cast<JetType>();
    BiasAccJet.x().v(10) = 1;
    BiasAccJet.y().v(11) = 1;
    BiasAccJet.z().v(12) = 1;

    VectorT<JetType,3> BiasTRJet = this->_BiasTR.template cast<JetType>();
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
    this->_Translation.x() = Translation2Jet.x().a;
    this->_Translation.y() = Translation2Jet.y().a;
    this->_Translation.z() = Translation2Jet.z().a;

    this->_Velocity.x() = Velocity2Jet.x().a;
    this->_Velocity.y() = Velocity2Jet.y().a;
    this->_Velocity.z() = Velocity2Jet.z().a;

    this->_Rotation.x() = Rotation2Jet.x().a;
    this->_Rotation.y() = Rotation2Jet.y().a;
    this->_Rotation.z() = Rotation2Jet.z().a;
    this->_Rotation.w() = Rotation2Jet.w().a;

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
    Covariance.topLeftCorner<10,10>() = this->_TVRCov;
    Covariance.bottomRightCorner<6,6>().topLeftCorner<3,3>() = AccelerationCov;
    Covariance.bottomRightCorner<3,3>() = TurnRateCov;

    /** propagate covariance */
    this->_TVRCov = Jacobian*Covariance*Jacobian.transpose();

    /** update bias jacobians */
    this->_JacTransBiasAcc.row(0) = Translation2Jet.x().v.template segment<3>(10);
    this->_JacTransBiasAcc.row(1) = Translation2Jet.y().v.template segment<3>(10);
    this->_JacTransBiasAcc.row(2) = Translation2Jet.z().v.template segment<3>(10);

    this->_JacVelBiasAcc.row(0) = Velocity2Jet.x().v.template segment<3>(10);
    this->_JacVelBiasAcc.row(1) = Velocity2Jet.y().v.template segment<3>(10);
    this->_JacVelBiasAcc.row(2) = Velocity2Jet.z().v.template segment<3>(10);

    this->_JacTransBiasTR.row(0) = Translation2Jet.x().v.template segment<3>(13);
    this->_JacTransBiasTR.row(1) = Translation2Jet.y().v.template segment<3>(13);
    this->_JacTransBiasTR.row(2) = Translation2Jet.z().v.template segment<3>(13);

    this->_JacVelBiasTR.row(0) = Velocity2Jet.x().v.template segment<3>(13);
    this->_JacVelBiasTR.row(1) = Velocity2Jet.y().v.template segment<3>(13);
    this->_JacVelBiasTR.row(2) = Velocity2Jet.z().v.template segment<3>(13);

    this->_JacRotBiasTR.row(0) = Rotation2Jet.x().v.template segment<3>(13);
    this->_JacRotBiasTR.row(1) = Rotation2Jet.y().v.template segment<3>(13);
    this->_JacRotBiasTR.row(2) = Rotation2Jet.z().v.template segment<3>(13);
    this->_JacRotBiasTR.row(3) = Rotation2Jet.w().v.template segment<3>(13);

    /** update time variables */
    _CurrentTime = Timestamp;
    _DeltaTime += dt;
  }
}
