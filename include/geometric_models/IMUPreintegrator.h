/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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
      std::vector<libRSF::SensorData> Measurements;
  };

  class IMUPreintegrator
  {
    public:
      IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                       const double RandomWalkAcc, const double RandomWalkGyro,
                       const double CurrentTime);

      IMUPreintegrator(const Vector3 &BiasAcc, const Vector3 &BiasTR,
                       const double NoiseDensityAcc, const double NoiseDensityGyro,
                       const double RandomWalkAcc, const double RandomWalkGyro,
                       const double CurrentTime);

      virtual ~IMUPreintegrator() = default;

      void addMeasurement(const SensorData &IMUMeasurement);
      void integrateToTime(const double Timestamp);
      void updateBias(const Vector3 &BiasAcc, const Vector3 &BiasTR);

      PreintegratedIMUResult getPreintegratedState();

    private:
      void initialize();
      void integrateSingleMeasurement(const int Index, const double Timestamp);
      void integrateFull(const double Timestamp);

      /** measurements */
      std::vector<libRSF::SensorData> _Measurements;

      /** time stamp that marks the end of the integration */
      double _CurrentTime;

      /** IMU Noise parameters */
      const double _NoiseDensityAcc;
      const double _NoiseDensityGyro;
      const double _RandomWalkAcc;
      const double _RandomWalkGyro;

      /** measurement biases (fixed during integration) */
      Vector3 _BiasAcc;
      Vector3 _BiasTR;

      /** pre-integration objects (no direct physical meaning)*/
      Vector3 _Translation;
      Vector3 _Velocity;
      Quaternion _Rotation;

      /** pre-integrated covariance matrix [Translation, Velocity, Rotation]*/
      Matrix1010 _TVRCov;

      /** Jacobians of the pre-integrated objects w.r.t. the measurement biases*/
      Matrix33 _JacTransBiasAcc;
      Matrix33 _JacTransBiasTR;

      Matrix33 _JacVelBiasAcc;
      Matrix33 _JacVelBiasTR;

      Matrix43 _JacRotBiasTR;

      /** complete length of the pre-integrated trajectory [s] */
      double _DeltaTime;
  };
}

#endif // IMUPREINTEGRATION_H
