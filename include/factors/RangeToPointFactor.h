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
 * @file RangeToPointFactor.h
 * @author Tim Pfeifer
 * @date 9. Jun 2022
 * @brief A factor that represents a range measurement to a unknown point with a unique ID.
 * @copyright GNU Public License.
 *
 */

#ifndef RANGETOPOINTFACTOR_H
#define RANGETOPOINTFACTOR_H

#include "BaseFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class RangeToPointFactorBase : public BaseFactor<ErrorType, true, false, Dim, Dim>
  {
    public:
      /** construct factor and store measurement */
     RangeToPointFactorBase(ErrorType &Error, const Data &Range)
      {
        this->Error_ = Error;
        this->MeasurementVector_ = Range.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 1> Evaluate(const T* const Position1,
                             const T* const Position2,
                             const double &Range) const
      {
        VectorT<T, 1> Error;
        Error(0) = VectorDistance<Dim, T, T>(Position1, Position2)
                   - Range;
        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Position1,
                      const T* const Position2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Position1,
                                                              Position2,
                                               this->MeasurementVector_(0)),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::RangeToPoint2, ErrorType> {using Type = RangeToPointFactorBase<ErrorType, 2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::RangeToPoint3, ErrorType> {using Type = RangeToPointFactorBase<ErrorType, 3>;};
}

#endif // RANGETOPOINTFACTOR_H
