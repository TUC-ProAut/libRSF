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
 * @file BearingRangeFactor.h
 * @author Tim Pfeifer
 * @date 19.03.2021
 * @brief A factor that is able to represent a bearing range measurement. Either to a fixed point or to a landmark.
 * @copyright GNU Public License.
 *
 */

#ifndef BEARINGRANGEFACTOR_H
#define BEARINGRANGEFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"
#include "../NormalizeAngle.h"

namespace libRSF
{
  template <typename ErrorType>
  class BetweenBearingRange2Factor : public BaseFactor<ErrorType, true, false, 2, 1, 2>
  {
    public:

      /** construct factor and store measurement */
      BetweenBearingRange2Factor(ErrorType &Error, const Data &Measurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(2);
        this->MeasurementVector_ = Measurement.getMean();
      }

      /** deterministic error model */
      template <typename T>
      VectorT<T, 2> Evaluate(const T* const P1,
                             const T* const Yaw1,
                             const T* const P2) const
      {
        /** map to vectors */
        VectorRefConst<T, 2> Point1(P1);
        VectorRefConst<T, 2> Point2(P2);
        VectorRefConst<T, 1> Angle1(Yaw1);

        /** get relative translation */
        VectorT<T, 2> RelativeTrans = Point2 - Point1;

        /** get bearing and range */
        VectorT<T, 2> EstimatedBR;
        EstimatedBR(0) = NormalizeAngle(ceres::atan2(RelativeTrans(1), RelativeTrans(0)) - Angle1(0));
        EstimatedBR(1) = RelativeTrans.norm();

        return EstimatedBR - this->MeasurementVector_;
      }

      /** combine probabilistic and deterministic model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Point1,
                      const T* const Yaw1,
                      const T* const Point2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Point1, Yaw1, Point2),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        /** pointer to eigen objects */
        VectorRefConst<double, 2> Point1(StatePointers.at(0));
        const libRSF::Rotation2D Rot1(StatePointers.at(1)[0]);
        VectorRef<double, 2> Point2(StatePointers.at(2));

        /** separate measurement */
        libRSF::Vector2 PointRange;
        PointRange << this->MeasurementVector_(1), 0.0;
        const libRSF::Rotation2D RotBearing(this->MeasurementVector_(0));

        /** convert to Cartesian position */
        Point2 = Point1 + Rot1 * RotBearing * PointRange;
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::BetweenBearingRange2, ErrorType> {using Type = BetweenBearingRange2Factor<ErrorType>;};
}

#endif // BEARINGRANGEFACTOR_H
