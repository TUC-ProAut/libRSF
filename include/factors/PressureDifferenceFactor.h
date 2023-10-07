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
 * @file PressureDifferenceFactor.h
 * @author Tim Pfeifer
 * @date 22.03.2021
 * @brief A factor that connects the height of two points using a differential barometric pressure. The estimation is done at sea level.
 * @copyright GNU Public License.
 *
 */

#ifndef PRESSUREDIFFERENCEFACTOR_H
#define PRESSUREDIFFERENCEFACTOR_H

#include "BaseFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class PressureDifferenceFactorBase : public BaseFactor<ErrorType, true, false, Dim, Dim>
  {
  public:
    /** construct factor and store measurement */
    PressureDifferenceFactorBase(ErrorType &Error, const Data &PressureMeasurement)
    {
      this->Error_ = Error;
      this->MeasurementVector_.resize(1);
      this->MeasurementVector_ = PressureMeasurement.getMean();
    }

    /** deterministic error model */
    template <typename T>
    VectorT<T, 1> Evaluate(const T* const Point1,
                           const T* const Point2) const
    {
      VectorT<T,1> EstimatedPressureDiff;
      EstimatedPressureDiff(0) = HeightToPressure(Point2[Dim-1]) - HeightToPressure(Point1[Dim-1]);

      return  EstimatedPressureDiff - this->MeasurementVector_.template cast<T>();
    }

    /** combine probabilistic and deterministic model */
    template <typename T, typename... ParamsType>
    bool operator()(const T* const Point1,
                    const T* const Point2,
                    ParamsType... Params) const
    {
      return this->Error_.template weight<T>(this->Evaluate(Point1, Point2),
                                             Params...);
    }

    /** predict the next state for initialization */
  //      void predict(const std::vector<double*> &StatePointers) const
  //      {
  //        /** map pointer to vectors */
  //        VectorRefConst<double, Dim> State1(StatePointers.at(0));
  //        VectorRef<double, Dim> State2(StatePointers.at(1));
  //
  //        State2 = State1 + this->MeasurementVector_;
  //      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::PressureDiff2, ErrorType>
  {
    using Type = PressureDifferenceFactorBase<ErrorType,2>;
  };
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::PressureDiff3, ErrorType>
  {
    using Type = PressureDifferenceFactorBase<ErrorType,3>;
  };
}

#endif // PRESSUREDIFFERENCEFACTOR_H
