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
 * @file ConstantDriftFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two values with a constant drift model.
 * @copyright GNU Public License.
 *
 */

#ifndef CONSTANTDRIFTFACTOR_H
#define CONSTANTDRIFTFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class ConstantDriftFactorBase : public BaseFactor<ErrorType, false, true, Dim, Dim, Dim, Dim>
  {
    public:
      /** construct factor and store measurement */
      ConstantDriftFactorBase(ErrorType &Error, double DeltaTime)
      {
        this->Error_ = Error;
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      MatrixT<T, Dim * 2, 1> Evaluate(const T* const ValueOld, const T* const DriftOld,
                                      const T* const ValueNew, const T* const DriftNew) const
      {
        VectorRefConst<T, Dim> V1(ValueOld);
        VectorRefConst<T, Dim> D1(DriftOld);
        VectorRefConst<T, Dim> V2(ValueNew);
        VectorRefConst<T, Dim> D2(DriftNew);

        VectorT<T, Dim * 2> Error;

        Error.template head<Dim>() = V2 - V1 - (D1 * this->DeltaTime_);
        Error.template tail<Dim>() = D2 - D1;

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const ValueOld, const T* const DriftOld,
                      const T* const ValueNew, const T* const DriftNew,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(ValueOld, DriftOld,
                                                              ValueNew, DriftNew),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        VectorRefConst<double, Dim> V1(StatePointers.at(0));
        VectorRefConst<double, Dim> D1(StatePointers.at(1));
        VectorRef<double, Dim> V2(StatePointers.at(2));
        VectorRef<double, Dim> D2(StatePointers.at(3));

        V2 = V1 + (D1 * this->DeltaTime_);
        D2 = D1;
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstDrift1, ErrorType> {using Type = ConstantDriftFactorBase<ErrorType, 1>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstDrift2, ErrorType> {using Type = ConstantDriftFactorBase<ErrorType, 2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::ConstDrift3, ErrorType> {using Type = ConstantDriftFactorBase<ErrorType, 3>;};
}
#endif // CONSTANTDRIFTFACTOR_H
