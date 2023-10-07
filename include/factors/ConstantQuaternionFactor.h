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
 * @file ConstantQuaternion.h
 * @author Tim Pfeifer
 * @date 02.03.2021
 * @brief A Factor that connects two quaternions with a body-frame rotation.
 * @copyright GNU Public License.
 *
 */

#ifndef CONSTANTQUATERNIONFACTOR_H
#define CONSTANTQUATERNIONFACTOR_H

#include "../Geometry.h"
#include "BaseFactor.h"

namespace libRSF
{
  template <typename ErrorType>
  class ConstantQuaternionFactor : public BaseFactor<ErrorType, false, false, 4, 4>
  {
   public:
    /** construct factor and store measurement */
    explicit ConstantQuaternionFactor(ErrorType &Error)
    {
      this->Error_ = Error;
    }

    /** geometric error model */
    template <typename T>
    VectorT<T, 3> Evaluate(const T * const Quat1, const T * const Quat2) const
    {
      /** map states */
      QuaternionRefConst<T> Q1(Quat1);
      QuaternionRefConst<T> Q2(Quat2);

      return QuaternionError<T>(Q1, Q2);
    }

    /** combine probabilistic and geometric model */
    template <typename T, typename... ParamsType>
    bool operator()(const T * const Quat1,
                    const T * const Quat2,
                    ParamsType... Params) const
    {
        return this->Error_.template weight<T>(this->Evaluate(Quat1,
                                                              Quat2),
                                               Params...);
    }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template <typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstQuaternion, ErrorType>
  {
    using Type = ConstantQuaternionFactor<ErrorType>;
  };
}  // namespace libRSF

#endif  // CONSTANTQUATERNIONFACTOR_H
