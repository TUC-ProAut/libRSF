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
 * @file PriorFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A factor that is able to represent prior knowledge or absolute measurements.
 * @copyright GNU Public License.
 *
 */

#ifndef PRIORFACTOR_H
#define PRIORFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"
#include "../Geometry.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>  class PriorFactorBase : public BaseFactor<ErrorType, true, false, Dim>
  {
    public:
      /** construct factor and store measurement */
      PriorFactorBase(ErrorType &Error, const SensorData &PriorMeasurement)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(Dim);
        this->_MeasurementVector = PriorMeasurement.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, Dim> Evaluate(const T* const StatePointer, const VectorStatic<Dim> &PriorValue) const
      {
        VectorRefConst<T, Dim> State(StatePointer);

        return State - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const State, ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(State, this->_MeasurementVector),
                                               Params...);
      }
  };


  template <typename ErrorType>
  class PriorFactorAngle : public BaseFactor<ErrorType, true, false, 1>
  {
    public:
      /** construct factor and store measurement */
      PriorFactorAngle(ErrorType &Error, const SensorData &PriorAngle)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(1);
        this->_MeasurementVector = PriorAngle.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 1> Evaluate(const T* const Angle, const double PriorAngle) const
      {
        return (VectorT<T, 1>() << NormalizeAngle(*Angle - PriorAngle)).finished();
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const State, ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(State, this->_MeasurementVector(0)),
                                               Params...);
      }
  };

  template <typename ErrorType>
  class PriorFactorQuaternion : public BaseFactor<ErrorType, true, false, 4>
  {
    public:
      /** construct factor and store measurement */
      PriorFactorQuaternion(ErrorType &Error, const SensorData &PriorQuaternion)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(4);
        this->_MeasurementVector = PriorQuaternion.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 3> Evaluate(const T* const Quaternion, const Vector4 &PriorQuaternion) const
      {
        QuaternionRefConst<T> Q1(Quaternion);
        libRSF::Quaternion Q2(PriorQuaternion(3), PriorQuaternion(0), PriorQuaternion(1), PriorQuaternion(2));

        return QuaternionError<T>(Q1, Q2.template cast<T>());
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const State, ParamsType... Params) const
      {
        return this->_Error.template weight<T>(this->Evaluate(State, this->_MeasurementVector),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Prior1, ErrorType> {using Type = PriorFactorBase<ErrorType, 1>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Prior2, ErrorType> {using Type = PriorFactorBase<ErrorType, 2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Prior3, ErrorType> {using Type = PriorFactorBase<ErrorType, 3>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Prior4, ErrorType> {using Type = PriorFactorBase<ErrorType, 4>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Prior9, ErrorType> {using Type = PriorFactorBase<ErrorType, 9>;};

  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::PriorQuat, ErrorType> {using Type = PriorFactorQuaternion<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::PriorAngle, ErrorType> {using Type = PriorFactorAngle<ErrorType>;};

}

#endif // PRIORFACTOR_H
