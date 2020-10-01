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
 * @file ConstantDriftFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two values with a constant value model.
 * @copyright GNU Public License.
 *
 */

#ifndef CONSTANTVALUEFACTOR_H
#define CONSTANTVALUEFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class ConstantValueFactorBase : public BaseFactor<ErrorType, false, false, Dim, Dim>
  {
    public:
      /** construct factor and store error model */
      ConstantValueFactorBase(ErrorType &Error)
      {
        this->_Error = Error;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, Dim> Evaluate(const T* const Value1, const T* const Value2) const
      {
        VectorRefConst<T, Dim> V1(Value1);
        VectorRefConst<T, Dim> V2(Value2);

        return V2 - V1;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const ValueOld,
                      const T* const ValueNew,
                      ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(ValueOld,
                                               ValueNew),
                                               Params...);

      }

  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstVal1, ErrorType> {using Type = ConstantValueFactorBase<ErrorType, 1>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstVal2, ErrorType> {using Type = ConstantValueFactorBase<ErrorType, 2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstVal3, ErrorType> {using Type = ConstantValueFactorBase<ErrorType, 3>;};
}

#endif // CONSTANTVALUEFACTOR_H
