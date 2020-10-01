/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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

/**
 * @file OdometryIntegrator.h
 * @author Tim Pfeifer
 * @date 12.05.2020
 * @brief Integrates odometry measurements to a relative pose in 3D
 * @copyright GNU Public License.
 *
 */

#ifndef ODOMETRYINTEGRATOR_H
#define ODOMETRYINTEGRATOR_H

#include "OdometryModel.h"
#include "../SensorData.h"

namespace libRSF
{
  class OdometryIntegrator
  {
    public:

      OdometryIntegrator();
      virtual ~OdometryIntegrator() = default;

      /** init */
      void reset();

      /** integrate measurements */
      void addMeasurement(const Vector3 &Velocity, const Vector3 &TurnRate,
                          const Vector3 &VelocityCov, const Vector3 &TurnRateCov,
                          const double DeltaTime);
      void addMeasurement(const SensorData &Odom, const double DeltaTime);

      /** get results */
      void getPose(Vector3 &Translation, Quaternion &Rotation) const;
      void getCov(Matrix33 &TranslationCov, Matrix44 &RotationCov) const;
      void getCovOnManifold(Matrix33 &TranslationCov, Matrix33 &RotationCov) const;

      Vector7 getJointPose() const;
      Matrix66 getJointCovOnManifold() const;

      double getTime() const;


    private:

      /** sum of the integrated time */
      double _Time;

      /** mean */
      Vector3 _Translation;
      Quaternion _Rotation;

      /** uncertainty */
      Matrix77 _PoseCov;
  };
}

#endif // ODOMETRYINTEGRATOR_H
