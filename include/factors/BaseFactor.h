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
 * @file BaseFactor.h
 * @author Tim Pfeifer
 * @date 12.03.2018
 * @brief Factor base class that separates the probabilistic error model from the physical sensor model
 * @copyright GNU Public License.
 *
 */

#ifndef BASEFACTOR_H
#define BASEFACTOR_H

#include "../Messages.h"
#include "../Data.h"
#include "../Types.h"
#include "../error_models/ErrorModel.h"

using ceres::AutoDiffCostFunction;

namespace libRSF
{
  /** compile time mapping from factor type enum to the corresponding class */
  template<FactorType Factor, typename ErrorType>
  struct FactorTypeTranslator;

  template <typename ErrorType,
            bool HasMeasurementTemp,
            bool HasDeltaTimeTemp,
            int... StateDimsTemp>   // Number of parameters
  class BaseFactor
  {
    public:

      /** Default constructor */
      BaseFactor() = default;

      /** Default destructor */
      virtual ~BaseFactor() = default;

      /** access the stored error model */
      ErrorType* getErrorModel()
      {
        return &Error_;
      }

      /** empty generic predictor */
      virtual void predict(const std::vector<double*> &States) const {};

      /** store the dimension of variables at compile time */
      using StateDims = std::integer_sequence<int, StateDimsTemp...>;

      /** configuration options */
      const static bool HasDeltaTime = HasDeltaTimeTemp;
      const static bool HasMeasurement = HasMeasurementTemp;

    protected:
      ErrorType   Error_; /**< represent the probabilistic error function */

      /** optional members */
      double      DeltaTime_ = 0;     /**< the time that has been past between the connected states */
      Vector      MeasurementVector_; /**< a efficient representation of the measurement */
  };
}

#endif // BASEFACTOR_H
