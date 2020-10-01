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
 * @file BetweenValueFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two values with a relative measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef BETWEENVALUEFACTOR_H
#define BETWEENVALUEFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class BetweenValueFactorBase : public BaseFactor<ErrorType, true, false, Dim, Dim>
  {
    public:
      /** construct factor and store measurement */
      BetweenValueFactorBase(ErrorType &Error, const SensorData &Measurement)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(Dim);
        this->_MeasurementVector = Measurement.getMean();
      }

      /** deterministic error model */
      template <typename T>
      VectorT<T, Dim> Evaluate(const T* const Value1,
                                      const T* const Value2,
                                      const VectorStatic<Dim> &BetweenValue) const
      {
        VectorRefConst<T, Dim> V1(Value1);
        VectorRefConst<T, Dim> V2(Value2);

        return V2 - V1 - BetweenValue.template cast<T>();
      }

      /** combine probabilistic and deterministic model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Value1,
                      const T* const Value2,
                      ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(Value1,
                                                              Value2,
                                                              this->_MeasurementVector),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::BetweenValue1, ErrorType> {using Type = BetweenValueFactorBase<ErrorType,1>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::BetweenValue2, ErrorType> {using Type = BetweenValueFactorBase<ErrorType,2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::BetweenValue3, ErrorType> {using Type = BetweenValueFactorBase<ErrorType,3>;};
}

#endif // BETWEENVALUEFACTOR_H
