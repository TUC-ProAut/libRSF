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

#ifndef TRACKINGDETECTIONFACTOR_H
#define TRACKINGDETECTIONFACTOR_H

#include "BaseFactor.h"
#include "../VectorMath.h"
#include "../Geometry.h"

namespace libRSF
{
  /**< pure detection factor, only with position*/
  template <typename ErrorType>
  class TrackingDetectionFactor : public BaseFactor<ErrorType, true, false, 3>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(3);
        this->MeasurementVector_ = PriorMeasurement.getMean().head(3);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 3> Evaluate(const T* const PositionStatePointer,
                             const Vector3 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        return PositionState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< partial factor only with rotation */
  template <typename ErrorType>
  class TrackingDetectionRotFactor : public BaseFactor<ErrorType, true, false, 3, 2>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionRotFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(5);
        this->MeasurementVector_ << PriorMeasurement.getMean().head(3), PriorMeasurement.getMean().tail(2);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 5> Evaluate(const T* const PositionStatePointer,
                             const T* const RotationStatePointer,
                             const Vector5 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 2> RotationState(RotationStatePointer);

        VectorT<T,5> FullState;
        FullState << PositionState, RotationState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const RotationState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              RotationState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< partial factor only with dimensions*/
  template <typename ErrorType>
  class TrackingDetectionDimFactor : public BaseFactor<ErrorType, true, false, 3, 3>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionDimFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(6);
        this->MeasurementVector_ << PriorMeasurement.getMean().head(3), PriorMeasurement.getMean().segment(6,3);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 6> Evaluate(const T* const PositionStatePointer,
                              const T* const BoxDimStatePointer,
                              const Vector6 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> BoxDimState(BoxDimStatePointer);

        VectorT<T,6> FullState;
        FullState << PositionState, BoxDimState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const BoxDimState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              BoxDimState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< partial factor only with velocity*/
  template <typename ErrorType>
  class TrackingDetectionVelFactor : public BaseFactor<ErrorType, true, false, 3, 3>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionVelFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(6);
        this->MeasurementVector_ = PriorMeasurement.getMean().head(6);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 6> Evaluate(const T* const PositionStatePointer,
                             const T* const VelocityStatePointer,
                             const Vector6 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> VelocityState(VelocityStatePointer);

        VectorT<T,6> FullState;
        FullState << PositionState, VelocityState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const VelocityState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              VelocityState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< partial factor with dimensions and rotation */
  template <typename ErrorType>
  class TrackingDetectionDimRotFactor : public BaseFactor<ErrorType, true, false, 3, 3, 2>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionDimRotFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(8);
        this->MeasurementVector_ << PriorMeasurement.getMean().head(3), PriorMeasurement.getMean().tail(5);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 8> Evaluate(const T* const PositionStatePointer,
                              const T* const BoxDimStatePointer,
                              const T* const RotationStatePointer,
                              const Vector8 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> BoxDimState(BoxDimStatePointer);
        VectorRefConst<T, 2> RotationState(RotationStatePointer);

        VectorT<T,8> FullState;
        FullState << PositionState, BoxDimState, RotationState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const BoxDimState,
                      const T* const RotationState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              BoxDimState,
                                                              RotationState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< partial factor with velocity and rotation */
  template <typename ErrorType>
  class TrackingDetectionVelRotFactor : public BaseFactor<ErrorType, true, false, 3, 3, 2>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionVelRotFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(8);
        this->MeasurementVector_ << PriorMeasurement.getMean().head(6), PriorMeasurement.getMean().tail(2);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 8> Evaluate(const T* const PositionStatePointer,
                              const T* const VelocityStatePointer,
                              const T* const RotationStatePointer,
                              const Vector8 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> VelocityState(VelocityStatePointer);
        VectorRefConst<T, 2> RotationState(RotationStatePointer);

        VectorT<T,8> FullState;
        FullState << PositionState, VelocityState, RotationState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const VelocityState,
                      const T* const RotationState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              VelocityState,
                                                              RotationState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< partial factor with velocity and dimensions */
  template <typename ErrorType>
  class TrackingDetectionVelDimFactor : public BaseFactor<ErrorType, true, false, 3, 3, 3>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionVelDimFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(9);
        this->MeasurementVector_ = PriorMeasurement.getMean().head(9);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 9> Evaluate(const T* const PositionStatePointer,
                              const T* const VelocityStatePointer,
                              const T* const BoxDimStatePointer,
                              const Vector9 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> VelocityState(VelocityStatePointer);
        VectorRefConst<T, 3> BoxDimState(BoxDimStatePointer);

        VectorT<T,9> FullState;
        FullState << PositionState, VelocityState, BoxDimState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const VelocityState,
                      const T* const BoxDimState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              VelocityState,
                                                              BoxDimState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /**< complete factor with velocity, dimensions and rotation */
  template <typename ErrorType>
  class TrackingDetectionVelDimRotFactor : public BaseFactor<ErrorType, true, false, 3, 3, 3, 2>
  {
    public:
      /** construct factor and store error model */
      TrackingDetectionVelDimRotFactor(ErrorType &Error, const Data &PriorMeasurement)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(11);
        this->MeasurementVector_ = PriorMeasurement.getMean();
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 11> Evaluate(const T* const PositionStatePointer,
                              const T* const VelocityStatePointer,
                              const T* const BoxDimStatePointer,
                              const T* const RotationStatePointer,
                              const Vector11 &PriorValue) const
      {
        VectorRefConst<T, 3> PositionState(PositionStatePointer);
        VectorRefConst<T, 3> VelocityState(VelocityStatePointer);
        VectorRefConst<T, 3> BoxDimState(BoxDimStatePointer);
        VectorRefConst<T, 2> RotationState(RotationStatePointer);

        VectorT<T,11> FullState;
        FullState << PositionState, VelocityState, BoxDimState, RotationState;

        return FullState - PriorValue;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const PositionState,
                      const T* const VelocityState,
                      const T* const BoxDimState,
                      const T* const RotationState,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(PositionState,
                                                              VelocityState,
                                                              BoxDimState,
                                                              RotationState,
                                                              this->MeasurementVector_),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetection, ErrorType> {using Type = TrackingDetectionFactor<ErrorType>;};

  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionVel, ErrorType> {using Type = TrackingDetectionVelFactor<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionDim, ErrorType> {using Type = TrackingDetectionDimFactor<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionRot, ErrorType> {using Type = TrackingDetectionRotFactor<ErrorType>;};

  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionVelDim, ErrorType> {using Type = TrackingDetectionVelDimFactor<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionVelRot, ErrorType> {using Type = TrackingDetectionVelRotFactor<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionDimRot, ErrorType> {using Type = TrackingDetectionDimRotFactor<ErrorType>;};

  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::TrackingDetectionVelDimRot, ErrorType> {using Type = TrackingDetectionVelDimRotFactor<ErrorType>;};
}

#endif // TRACKINGDETECTIONFACTOR_H
