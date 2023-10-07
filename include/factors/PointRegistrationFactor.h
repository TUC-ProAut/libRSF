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
 * @file PointRegistrationFactor.h
 * @author Tim Pfeifer
 * @date 18.12.2019
 * @brief A Factor that connects two poses by registering two measured points.
 * @copyright GNU Public License.
 *
 */

#ifndef POINTREGISTRATIONFACTOR_H
#define POINTREGISTRATIONFACTOR_H

#include "BaseFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType>
  class Point2RegistrationFactor : public BaseFactor<ErrorType, true, false, 2, 1, 2, 1>
  {
    public:
      /** construct factor and store measurement */
      Point2RegistrationFactor(ErrorType &Error, const Data &RelativePoint)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(4);
        this->MeasurementVector_ = RelativePoint.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 2> Evaluate(const T* const Pos1, const T* const Yaw1,
                                const T* const Pos2, const T* const Yaw2,
                                const Vector2 &Point1, const Vector2 &Point2) const
      {
        /** map to eigen objects */
        VectorRefConst<T, 2> T1(Pos1);
        VectorRefConst<T, 2> T2(Pos2);

        const Eigen::Rotation2D<T> Rot1Inv(-Yaw1[0]);
        const Eigen::Rotation2D<T> Rot2(Yaw2[0]);

        /** calculate error in frame 1 */
        VectorT<T, 2> Error;
        Error = Point1.template cast<T>() + Rot1Inv * (T1 - T2 - Rot2 * Point2.template cast<T>());

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Yaw1, const T* const Pos2, const T* const Yaw2, ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Yaw1,
                                               Pos2, Yaw2,
                                               this->MeasurementVector_.head(2),
                                               this->MeasurementVector_.tail(2)),
                                               Params...);
      }
  };

  template <typename ErrorType>
  class Point2RegistrationPoseFactor : public BaseFactor<ErrorType, true, false, 3, 3>
  {
    public:
      /** construct factor and store measurement */
      Point2RegistrationPoseFactor(ErrorType &Error, const Data &RelativePoint)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(4);
        this->MeasurementVector_ = RelativePoint.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 2> Evaluate(const T* const Pose1, const T* const Pose2,
                             const Vector2 &Point1, const Vector2 &Point2) const
      {
        /** map to eigen objects */
        VectorRefConst<T, 2> T1(Pose1);
        VectorRefConst<T, 2> T2(Pose2);

        const Eigen::Rotation2D<T> Rot1Inv(-Pose1[2]);
        const Eigen::Rotation2D<T> Rot2(Pose2[2]);

        /** calculate error in frame 1 */
        VectorT<T, 2> Error;
        Error = Point1.template cast<T>() + Rot1Inv * (T1 - T2 - Rot2 * Point2.template cast<T>());

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pose1, const T* const Pose2, ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pose1, Pose2,
                                               this->MeasurementVector_.head(2),
                                               this->MeasurementVector_.tail(2)),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Point2Reg, ErrorType> {using Type = Point2RegistrationFactor<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Point2RegPose, ErrorType> {using Type = Point2RegistrationPoseFactor<ErrorType>;};
}

#endif // POINTREGISTRATIONFACTOR_H
