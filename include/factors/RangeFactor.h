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
 * @file RangeFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A factor that represents a range measurement to a fixed point.
 * @copyright GNU Public License.
 *
 */

#ifndef RANGEFACTOR_H
#define RANGEFACTOR_H

#include <ceres/ceres.h>
#include "MeasurementFactor.h"
#include "../Geometry.h"

namespace libRSF
{
  template <int Dimensions>
  SensorType RangeType();

  template <int Dimensions>
  class RangeModel : public SensorModel
  {
    public:
      RangeModel()
      {
        _OutputDim = 1;
        _InputDim = Dimensions + 1;
      }

      template <typename T>
      bool Evaluate(const T* const Position1, const ceres::Vector &Position2, const ceres::Vector &Range,  T* Error) const
      {
        Error[0] = VectorDistance<Dimensions, T, double>(Position1, Position2.data())
                   - T(Range[0]);
        return true;
      }
  };

  template <typename ErrorType, int Dimensions>
  class RangeFactorBase : public MeasurementFactor<RangeModel<Dimensions>, ErrorType, Dimensions>
  {
    public:
      RangeFactorBase(ErrorType &Error, MeasurementList &Measurements)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(Dimensions + 1);
        this->_MeasurementVector[0] = Measurements.get(RangeType<Dimensions>()).getMean()[0];
        this->_MeasurementVector.tail(Dimensions) = Measurements.get(RangeType<Dimensions>()).getValue(SensorElement::SatPos);

        this->CheckInput();
      }

      /** normal version for static error models */
      template <typename T>
      bool operator()(const T* const Position,  T* Error) const
      {
        this->_Model.Evaluate(Position, this->_MeasurementVector.tail(Dimensions), this->_MeasurementVector.head(1), Error);
        this->_Error.Evaluate(Error);

        return true;
      }

      /** special version for SC and DCE */
      template <typename T>
      bool operator()(const T* const Position, const T* const ErrorModelState, T* Error) const
      {
        this->_Model.Evaluate(Position, this->_MeasurementVector.tail(Dimensions), this->_MeasurementVector.head(1), Error);
        this->_Error.Evaluate(ErrorModelState, Error);

        return true;
      }

  };

}

#endif // RANGEFACTOR_H
