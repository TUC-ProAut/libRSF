/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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
 * @file TrackingFactor.h
 * @author Johannes Poeschmann
 * @date 10.09.2019
 * @brief A Factor that connects two 2D poses with a relative pose measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef TRACKINGDETECTIONFACTOR_H
#define TRACKINGDETECTIONFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType>
  class TrackingDetectionFactor : public BaseFactor<ErrorType, true, false, 3, 3>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionFactor(ErrorType &Error, const SensorData &PriorMeasurement)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(6);
        this->_MeasurementVector = PriorMeasurement.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 6> Evaluate(const T* const PositionStatePointer, const T* const VelocityStatePointer, const VectorStatic<6> &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> VelocityState(VelocityStatePointer);
        return (VectorT<T, 6>() << PositionState - PriorValue.head(3), VelocityState - PriorValue.tail(3)).finished();
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState, const T* const VelocityState, ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(PositionState, VelocityState, this->_MeasurementVector),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetection, ErrorType> {using Type = TrackingDetectionFactor<ErrorType>;};
}

#endif // TRACKINGDETECTIONFACTOR_H
