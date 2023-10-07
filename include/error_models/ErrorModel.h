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
 * @file ErrorModel.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Base class for probabilistic error models like Gaussian or Max-Mixture.
 * @copyright GNU Public License.
 *
 */

#ifndef ERRORMODEL_H
#define ERRORMODEL_H

#include <utility> /**< for integer_sequence */

namespace libRSF
{
  class ErrorModelBase
  {
    public:
      ErrorModelBase() = default;
      virtual ~ErrorModelBase() = default;

      void enable()
      {
        Enable_ = true;
      }

      void disable()
      {
        Enable_ = false;
      }

    protected:
      bool Enable_ = true;
  };


  template <int InputDimTemp, int OutputDimTemp, int ...StateDimsTemp>
  class ErrorModel: public ErrorModelBase
  {
    public:
      ErrorModel() = default;
      ~ErrorModel() override = default;

      /** static access to dimensions*/
      static const int InputDim = InputDimTemp;
      static const int OutputDim = OutputDimTemp;

      /** store the dimension of variables at compile time */
      using StateDims = std::integer_sequence<int, StateDimsTemp...>;
  };
}

#endif // ERRORMODEL_H
