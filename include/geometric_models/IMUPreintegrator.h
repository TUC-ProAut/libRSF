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
 * @file IMUPreintegrator.h
 * @author Tim Pfeifer
 * @date 29.03.2019
 * @brief Creates an IMU pre-integration object.
 * @copyright GNU Public License.
 *
 */

#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H

#include "Geometry.h"
#include "SensorDataSet.h"

#include <Eigen/Dense>

namespace libRSF
{
  struct PreintegratedIMUResult
  {
     /** measurement biases (fixed during integration) */
      Vector3 BiasAcc;
      Vector3 BiasTR;

      /** pre-integration objects (no direct physical meaning)*/
      Vector3 Translation;
      Vector3 Velocity;
      Quaternion Rotation;

      /** pre-integrated covariance matrix [Translation, Velocity, Rotation, BiasAcc, BiasTR]*/
      MatrixStatic<15,15> PreIntCov;

      /** Jacobians of the pre-integrated objects w.r.t. the measurement biases*/
      Matrix33 JacTransBiasAcc;
      Matrix33 JacTransBiasTR;

      Matrix33 JacVelBiasAcc;
      Matrix33 JacVelBiasTR;

      /** Jacobians in local tangent space */
      Matrix33 JacRotBiasTRLocal;

      /** complete length of the pre-integrated trajectory [s] */
      double DeltaTime;

      /** start time */
      double StartTime;

      /** all IMU measurements */
      std::vector<libRSF::Data> Measurements;
  };

  class IMUPreintegrator
  {
    public:
      IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                       double RandomWalkAcc, double RandomWalkGyro,
                       double CurrentTime);

      IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                       double NoiseDensityAcc, double NoiseDensityGyro,
                       double RandomWalkAcc, double RandomWalkGyro,
                       double CurrentTime);

      virtual ~IMUPreintegrator() = default;

      void addMeasurement(const Data &IMUMeasurement);
      void integrateToTime(double Timestamp);
      void updateBias(const Vector3 &BiasAcc, const Vector3 &BiasTR);

      PreintegratedIMUResult getPreintegratedState();

    private:
      void initialize_();
      void integrateSingleMeasurement_(int Index, double Timestamp);
      void integrateFull_(double Timestamp);

      /** measurements */
      std::vector<libRSF::Data> Measurements_;

      /** time stamp that marks the end of the integration */
      double CurrentTime_;

      /** IMU Noise parameters */
      const double NoiseDensityAcc_;
      const double NoiseDensityGyro_;
      const double RandomWalkAcc_;
      const double RandomWalkGyro_;

      /** measurement biases (fixed during integration) */
      Vector3 BiasAcc_;
      Vector3 BiasTR_;

      /** pre-integration objects (no direct physical meaning)*/
      Vector3 Translation_;
      Vector3 Velocity_;
      Quaternion Rotation_;

      /** pre-integrated covariance matrix [Translation, Velocity, Rotation]*/
      Matrix1010 TVRCov_;

      /** Jacobians of the pre-integrated objects w.r.t. the measurement biases*/
      Matrix33 JacTransBiasAcc_;
      Matrix33 JacTransBiasTR_;

      Matrix33 JacVelBiasAcc_;
      Matrix33 JacVelBiasTR_;

      Matrix43 JacRotBiasTR_;

      /** complete length of the pre-integrated trajectory [s] */
      double DeltaTime_ = 0;
  };
}

#endif // IMUPREINTEGRATION_H
