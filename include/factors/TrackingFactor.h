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
 * @file TrackingFactor.h
 * @author Johannes Poeschmann
 * @date 10.09.2019
 * @brief A Factor that connects two 2D poses with a relative pose measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef TRACKINGFACTOR_H
#define TRACKINGFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"

namespace libRSF
{
  template <typename ErrorType>
  class RepellingFactor : public BaseFactor<ErrorType, false, false, 3, 3>
  {
    public:
      /** construct factor and store error model */
      explicit RepellingFactor(ErrorType &Error)
      {
        this->Error_ = Error;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 1> Evaluate(const T* const Value1, const T* const Value2) const
      {
        VectorRefConst<T, 3> V1(Value1);
        VectorRefConst<T, 3> V2(Value2);
        VectorT<T, 1> Error;

        Error(0) = 1.0 / ((V2 - V1).squaredNorm() + 1e-10); /**< added small constant for numerical stability */

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1,
                      const T* const Pos2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Pos2),
                                               Params...);

      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Repelling, ErrorType> {using Type = RepellingFactor<ErrorType>;};
}

#endif // TRACKINGFACTOR_H
