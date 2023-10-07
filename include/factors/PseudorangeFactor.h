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
 * @file PseudorangeFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A factor that represents a pseudo range measurement to a fixed point.
 * @copyright GNU Public License.
 *
 */

#ifndef PSEUDORANGEFACTOR_H
#define PSEUDORANGEFACTOR_H

#include "RangeFactor.h"
#include "BaseFactor.h"
#include "../Geometry.h"
#include "../Constants.h"
#include "../VectorMath.h"

namespace libRSF
{

  template <typename T>
  T RelativisticCorrection(const T* const EgoPos, const Vector3 &SatPos)
  {
    return (EARTH_ROTATION_RATE * (SatPos[0] * EgoPos[1] - SatPos[1] * EgoPos[0])) / SPEED_OF_LIGHT;
  }

  template <typename ErrorType, int Dim>
  class PseudorangeFactorBase : public BaseFactor< ErrorType, true, false, Dim, 1>
  {
    public:
      /** construct factor and store measurement */
      PseudorangeFactorBase(ErrorType &Error, const Data &Pseudorange)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(Dim + 1);
        this->MeasurementVector_[0] = Pseudorange.getMean()[0];
        this->MeasurementVector_.tail(Dim) = Pseudorange.getValue(DataElement::SatPos);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 1> Evaluate(const T* const EgoPos,
                                const T* const Offset,
                                const VectorStatic<Dim> &SatPos,
                                const double &Range) const
      {
        VectorT<T, 1> Error;
        Error(0) = VectorDistance<Dim, T, double>(EgoPos, SatPos.data())
                   + Offset[0]
                   - Range;
        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Position,
                      const T* const Offset,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Position,
                                               Offset,
                                               this->MeasurementVector_.tail(Dim),
                                               this->MeasurementVector_(0)),
                                               Params...);
      }
  };

  template <typename ErrorType, int Dim>
  class PseudorangeBiasFactorBase : public BaseFactor< ErrorType, true, false, Dim, 1, 1>
  {
   public:
    /** construct factor and store measurement */
    PseudorangeBiasFactorBase(ErrorType &Error, const Data &Pseudorange)
    {
      this->Error_ = Error;
      this->MeasurementVector_.resize(Dim + 1);
      this->MeasurementVector_[0] = Pseudorange.getMean()[0];
      this->MeasurementVector_.tail(Dim) = Pseudorange.getValue(DataElement::SatPos);
    }

    /** geometric error model */
    template <typename T>
    VectorT<T, 1> Evaluate(const T* const EgoPos,
                           const T* const Offset,
                           const T* const InterSystemBias,
                           const VectorStatic<Dim> &SatPos,
                           const double &Range) const
    {
      VectorT<T, 1> Error;
      Error(0) = VectorDistance<Dim, T, double>(EgoPos, SatPos.data())
                 + Offset[0]
                 + InterSystemBias[0]
                 - Range;
      return Error;
    }

    /** combine probabilistic and geometric model */
    template <typename T, typename... ParamsType>
    bool operator()(const T* const Position,
                    const T* const Offset,
                    const T* const InterSystemBias,
                    ParamsType... Params) const
    {
      return this->Error_.template weight<T>(this->Evaluate(Position,
                                                            Offset,
                                                            InterSystemBias,
                                                            this->MeasurementVector_.tail(Dim),
                                                            this->MeasurementVector_(0)),
                                             Params...);
    }
  };

  template <typename ErrorType, int Dim>
  class PseudorangeSagnacFactorBase : public BaseFactor<ErrorType, true, false, Dim, 1>
  {
    public:
      /** construct factor and store measurement */
      PseudorangeSagnacFactorBase(ErrorType &Error, const Data &Pseudorange)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(Dim + 1);
        this->MeasurementVector_[0] = Pseudorange.getMean()[0];
        this->MeasurementVector_.tail(Dim) = Pseudorange.getValue(DataElement::SatPos);
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 1> Evaluate(const T* const EgoPos,
                                const T* const Offset,
                                const VectorStatic<Dim> &SatPos,
                                const double &Range) const
      {
        VectorT<T, 1> Error;
        Error(0) = VectorDistance<Dim, T, double>(EgoPos, SatPos.data())
                   + RelativisticCorrection(EgoPos, SatPos)
                   + Offset[0]
                   - Range;
        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Position,
                      const T* const Offset,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Position, Offset,
                                               this->MeasurementVector_.tail(Dim),
                                               this->MeasurementVector_(0)),
                                               Params...);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Pseudorange2, ErrorType> {using Type = PseudorangeFactorBase<ErrorType, 2>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Pseudorange3, ErrorType> {using Type = PseudorangeFactorBase<ErrorType, 3>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Pseudorange3_Bias, ErrorType> {using Type = PseudorangeBiasFactorBase<ErrorType, 3>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Pseudorange3_ECEF, ErrorType> {using Type = PseudorangeSagnacFactorBase<ErrorType, 3>;};
}

#endif // PSEUDORANGEFACTOR_H
