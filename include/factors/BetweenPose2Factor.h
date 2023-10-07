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
 * @file BetweenPose2Factor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two 2D poses with a relative pose measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef BETWEENPOSE2FACTOR_H
#define BETWEENPOSE2FACTOR_H

#include "BaseFactor.h"
#include "../Geometry.h"
#include "../geometric_models/RelativePoseModel.h"

namespace libRSF
{
  template <typename ErrorType>
  class BetweenPose2Factor : public BaseFactor<ErrorType, true, false, 2, 1, 2, 1>
  {
    public:
      /** construct factor and store measurement */
      BetweenPose2Factor(ErrorType &Error, const Data &PoseMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(3);
        this->MeasurementVector_ = PoseMeasurement.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 3> Evaluate(const T* const Pos1, const T* const Yaw1,
                             const T* const Pos2, const T* const Yaw2,
                             const Vector3 &RelativePose) const
      {
        /** estimate measurements*/
        VectorT<T,2> TransEst;
        Rotation2DT<T> RotEst;
        RelativePose2Model<T>::applyBackward(Pos1, Yaw1,
                                             Pos2, Yaw2,
                                             TransEst, RotEst);

        /** error = estimated measurement - measurement */
        VectorT<T, 3> Error;
        Error.template head<2>() = TransEst - RelativePose.template head<2>().template cast<T>();
        Error(2) = NormalizeAngle<T>(RotEst.angle() - RelativePose(2));

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Yaw1,
                      const T* const Pos2, const T* const Yaw2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(
                                                              Pos1, Yaw1,
                                                              Pos2, Yaw2,
                                                              this->MeasurementVector_),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        /** split pose measurement */
        const Vector2 Trans = this->MeasurementVector_.head(2);
        const Rotation2D Rot(this->MeasurementVector_(2));

        /** apply forward model */
        RelativePose2Model<double>::applyForward(StatePointers.at(0), StatePointers.at(1),
                                                 StatePointers.at(2), StatePointers.at(3),
                                                 Trans, Rot);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::BetweenPose2, ErrorType> {using Type = BetweenPose2Factor<ErrorType>;};
}

#endif // BETWEENPOSE2FACTOR_H
