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
 * @file RangeFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A factor that represents a range measurement to a fixed point.
 * @copyright GNU Public License.
 *
 */

#ifndef RANGEFACTOR_H
#define RANGEFACTOR_H

#include "BaseFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class RangeFactorBase : public BaseFactor<ErrorType, true, false, Dim>
  {
    public:
      /** construct factor and store measurement */
      RangeFactorBase(ErrorType &Error, const Data &Range)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(Dim + 1);
        this->MeasurementVector_[0] = Range.getMean()[0];
        this->MeasurementVector_.tail(Dim) = Range.getValue(DataElement::SatPos);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 1> Evaluate(const T* const Position1,
                             const VectorStatic<Dim> &Position2,
                             const double &Range) const
      {
        VectorT<T, 1> Error;
        Error(0) = VectorDistance<Dim, T, double>(Position1, Position2.data())
                   - Range;
        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Position,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Position,
                                               this->MeasurementVector_.tail(Dim),
                                               this->MeasurementVector_(0)),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Range2, ErrorType> {using Type = RangeFactorBase<ErrorType, 2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Range3, ErrorType> {using Type = RangeFactorBase<ErrorType, 3>;};
}

#endif // RANGEFACTOR_H
