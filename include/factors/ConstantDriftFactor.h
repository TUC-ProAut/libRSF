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
 * @file ConstantDriftFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A Factor that connects two values with a constant drift model.
 * @copyright GNU Public License.
 *
 */

#ifndef CONSTANTDRIFTFACTOR_H
#define CONSTANTDRIFTFACTOR_H

#include <ceres/ceres.h>
#include "MeasurementFactor.h"
#include "../VectorMath.h"

namespace libRSF
{
  template <int Dimension>
  class ConstantDriftModel : public SensorModel
  {
    public:

      ConstantDriftModel()
      {
        _OutputDim = Dimension*2;
        _InputDim = 1;
      };

      template <typename T>
      bool Evaluate(const T* const ValueOld, const T* const ValueNew, const T* const DriftOld, const T* const DriftNew, const ceres::Vector DeltaTime,  T* Error) const
      {
        T PredictedValue[Dimension];

        for(int nDim = 0; nDim < Dimension; nDim++)
        {
          PredictedValue[nDim] = ValueOld[nDim] + DriftOld[nDim] * DeltaTime[0];
        }

        VectorDifference<Dimension, T>(PredictedValue, ValueNew, Error);
        VectorDifference<Dimension, T>(DriftOld, DriftNew, &(Error[Dimension]));
        return true;
      }
  };

  template <typename ErrorType, int Dimensions>
  class ConstantDriftFactorBase : public MeasurementFactor< ConstantDriftModel<Dimensions>, ErrorType, Dimensions, Dimensions, Dimensions, Dimensions>
  {
    public:

      ConstantDriftFactorBase(ErrorType &Error, MeasurementList &Measurements)
      {
        this->_Error = Error;
        this->_MeasurementVector.resize(1);
        this->_MeasurementVector = Measurements.get(SensorType::DeltaTime).getMean();

        this->CheckInput();
      }

      /** normal version for static error models */
      template <typename T>
      bool operator()(const T* const ValueOld, const T* const ValueNew, const T* const DriftOld, const T* const DriftNew, T* Error) const
      {
        this->_Model.Evaluate(ValueOld, ValueNew, DriftOld, DriftNew, this->_MeasurementVector, Error);
        this->_Error.Evaluate(Error);

        return true;
      }

      /** special version for SC and DCE */
      template <typename T>
      bool operator()(const T* const ValueOld, const T* const ValueNew, const T* const DriftOld, const T* const DriftNew, const T* const ErrorModelState, T* Error) const
      {
        this->_Model.Evaluate(ValueOld, ValueNew, DriftOld, DriftNew, this->_MeasurementVector, Error);
        this->_Error.Evaluate(ErrorModelState, Error);

        return true;
      }
  };

}
#endif // CONSTANTDRIFTFACTOR_H
