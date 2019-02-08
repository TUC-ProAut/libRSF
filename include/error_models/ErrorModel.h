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
 * @file ErrorModel.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Base class for probabilistic error models like Gaussian or Max-Mixture.
 * @copyright GNU Public License.
 *
 */

#ifndef ERRORMODEL_H
#define ERRORMODEL_H


#include <ceres/ceres.h>
#include "../StateData.h"

using ceres::CostFunctionToFunctor;
using ceres::Vector;

namespace libRSF
{
  class ErrorModelBase
  {
    public:
      ErrorModelBase() {};
      virtual ~ErrorModelBase() {};


      void enable()
      {
        _Enable = true;
      }

      void disable()
      {
        _Enable = false;
      }

    protected:
      bool _Enable = true;
  };


  template <int InputDim, int OutputDim, int ...StateDims>
  class ErrorModel: public ErrorModelBase
  {
    public:
      ErrorModel() {};
      virtual ~ErrorModel() {};

      const int getInputDim() const
      {
        return _InputDim;
      };

      const int getOutputDim() const
      {
        return _OutputDim;
      };

      static const int _OutputDim = OutputDim;
      static const int _InputDim = InputDim;

      /** store the dimension of variables at compile time */
      using _StateDims = std::integer_sequence<int, StateDims...>;
  };
}

#endif // ERRORMODEL_H
