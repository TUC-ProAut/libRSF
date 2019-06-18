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
 * @file PredictOdometry.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Predictors for 3D odometry models.
 * @copyright GNU Public License.
 *
 */

#ifndef PREDICTODOMETRY_H
#define PREDICTODOMETRY_H

#include <ceres/ceres.h>
#include "../Geometry.h"
#include "../VectorMath.h"

namespace libRSF
{

  ceres::Vector PredictMotionDifferential2D(ceres::Vector Pose, double VelLeft, double VelRight, double DeltaTime);

  template<typename T>
  void PredictOdometry4DOFGlobal(const T* PoseOld, T* PoseNew, const T* YawOld, T* YawNew, const ceres::Vector &Odometry, double DeltaTime)
  {
    Eigen::Matrix<T, 3, 1> ZAxis, Axis;
    Eigen::Matrix<T, 4, 1> QuatPose, QuatYaw, QuatOld;

    QuatYaw << cos(YawOld[0]/2.0), T(0), T(0), sin(YawOld[0]/2.0);
    ZAxis << T(0), T(0), T(1);

    ceres::CrossProduct(ZAxis.data(), PoseOld, Axis.data());

    QuatPose[0] = ceres::DotProduct(ZAxis.data(), PoseOld) + VectorLength<3>(ZAxis.data())*VectorLength<3>(PoseOld);
    QuatPose[1] = Axis[0];
    QuatPose[2] = Axis[1];
    QuatPose[3] = Axis[2];
    ceres::QuaternionProduct(QuatPose.data(), QuatYaw.data(), QuatOld.data());

    T Translation[3], TranslationRot[3];

    Translation[0] = T(Odometry[0]*DeltaTime);
    Translation[1] = T(Odometry[1]*DeltaTime);
    Translation[2] = T(Odometry[2]*DeltaTime);

    ceres::QuaternionRotatePoint(QuatOld.data(), Translation, TranslationRot);

    PoseNew[0] = TranslationRot[0] + PoseOld[0];
    PoseNew[1] = TranslationRot[1] + PoseOld[1];
    PoseNew[2] = TranslationRot[2] + PoseOld[2];

    YawNew[0] = NormalizeAngle(YawOld[0] + Odometry[5] * DeltaTime);
  }
}

#endif // PREDICTODOMETRY_H
