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
 * @file BetweenPose3Factor.h
 * @author Tim Pfeifer
 * @date 29.08.2019
 * @brief A Factor that connects two 3D poses with a relative pose measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef BETWEENPOSE3FACTOR_H
#define BETWEENPOSE3FACTOR_H

#include "BaseFactor.h"
#include "../Geometry.h"
#include "../geometric_models/RelativePoseModel.h"

namespace libRSF
{
  template <typename ErrorType>
  class BetweenPose3Factor : public BaseFactor<ErrorType, true, false, 3, 4, 3, 4>
  {
    public:
      /** construct factor and store measurement */
      BetweenPose3Factor(ErrorType &Error, const Data &PoseMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(7);
        this->MeasurementVector_ = PoseMeasurement.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 6> Evaluate(const T* const Pos1, const T* const Quat1,
                             const T* const Pos2, const T* const Quat2) const
      {
        /** store measurement separately */
        const Vector3 Translation = this->MeasurementVector_.head(3);
        const Quaternion QuatRot = VectorToQuaternion<double>(this->MeasurementVector_.tail(4));

        /** estimate measurements*/
        VectorT<T, 3> TransEst;
        QuaternionT<T> QuatEst;
        RelativePose3Model<T>::applyBackward(Pos1, Quat1,
                                             Pos2, Quat2,
                                             TransEst, QuatEst);

        /** error = estimated measurement - measurement */
        VectorT<T, 6> Error;
        Error.template head<3>() = TransEst - Translation.template cast<T>();
        Error.template tail<3>() = QuaternionError<T>(QuatRot.template cast<T>(), QuatEst);

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Quat1,
                      const T* const Pos2, const T* const Quat2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Quat1,
                                                              Pos2, Quat2),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        RelativePose3Model<double>::applyForward(StatePointers[0],
                                                 StatePointers[1],
                                                 StatePointers[2],
                                                 StatePointers[3],
                                                 this->MeasurementVector_.head(3),
                                                 VectorToQuaternion<double>(this->MeasurementVector_.tail(4)));
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::BetweenPose3, ErrorType> {using Type = BetweenPose3Factor<ErrorType>;};
}
#endif // BETWEENPOSE3FACTOR_H
