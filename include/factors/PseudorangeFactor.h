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
 * @file PseudorangeFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A factor that represents a pseudo range measurement to a fixed point.
 * @copyright GNU Public License.
 *
 */

#ifndef PSEUDORANGEFACTOR_H
#define PSEUDORANGEFACTOR_H

#include <ceres/ceres.h>
#include "MeasurementFactor.h"
#include "../Geometry.h"
#include "RangeFactor.h"

/** physical constants for sagnac effect */
#define EARTH_ROTATION_RATE   7.2921151467e-5
#define SPEED_OF_LIGHT        2.99792458e8

namespace libRSF
{

  template <typename T>
  T RelativisticCorrection(const T* const EgoPos, const ceres::Vector &SatPos)
  {
    return (EARTH_ROTATION_RATE * (SatPos[0] * EgoPos[1] - SatPos[1] * EgoPos[0])) / SPEED_OF_LIGHT;
  }

  template <int Dimensions>
  class PseudorangeModel : public SensorModel
  {
    public:
      PseudorangeModel()
      {
        _OutputDim = 1; /** one residual */
        _InputDim = Dimensions + 1; /** n-dimensional satellite position + 1D pseudorange */
      }

      template <typename T>
      bool Evaluate(const T* const EgoPos, const T* const Offset, const ceres::Vector &SatPos, const ceres::Vector &Range,  T* Error) const
      {
        Error[0] = VectorDistance<2, T, double>(EgoPos, SatPos.data())
                   + Offset[0]
                   - T(Range[0]);
        return true;
      }
  };

  template <>
  class PseudorangeModel<3> : public SensorModel
  {
    public:
      PseudorangeModel()
      {
        _OutputDim = 1; /** one residual */
        _InputDim = 4; /** 3D satellite position + 1D pseudorange */
      }

      template <typename T>
      bool Evaluate(const T* const EgoPos, const T* const Offset, const ceres::Vector &SatPos, const ceres::Vector &Range,  T* Error) const
      {
        Error[0] = VectorDistance<3, T, double>(EgoPos, SatPos.data())
                   + RelativisticCorrection(EgoPos, SatPos)
                   + Offset[0]
                   - Range[0];
        return true;
      }
  };

  template <typename ErrorType, int Dimensions>
  class PseudorangeFactorBase : public MeasurementFactor< PseudorangeModel<Dimensions>, ErrorType, Dimensions, 1>
  {
    public:

      PseudorangeFactorBase(ErrorType &Error, MeasurementList &Measurements)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(Dimensions + 1);
        this->_MeasurementVector[0] = Measurements.get(RangeType<Dimensions>()).getMean()[0];
        this->_MeasurementVector.tail(Dimensions) = Measurements.get(RangeType<Dimensions>()).getValue(SensorElement::SatPos);

        this->CheckInput();
      }

      /** normal version for static error models */
      template <typename T>
      bool operator()(const T* const EgoPos, const T* const Offset,  T* Error) const
      {
        this->_Model.Evaluate(EgoPos, Offset, this->_MeasurementVector.tail(Dimensions), this->_MeasurementVector.head(1), Error);
        this->_Error.Evaluate(Error);

        return true;
      }

      /** special version for SC and DCE */
      template <typename T>
      bool operator()(const T* const EgoPos, const T* const Offset, const T* const ErrorModelState,  T* Error) const
      {
        this->_Model.Evaluate(EgoPos, Offset, this->_MeasurementVector.tail(Dimensions), this->_MeasurementVector.head(1), Error);
        this->_Error.Evaluate(ErrorModelState, Error);

        return true;
      }

  };

}

#endif // PSEUDORANGEFACTOR_H
