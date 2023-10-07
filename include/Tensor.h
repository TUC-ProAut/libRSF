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
* @file Tensor.h
* @author Tim Pfeifer
* @date 19.05.2020
* @brief A very simplistic class to represent n-dimensional tensors. Just for storage.
* @copyright GNU Public License.
*
*/

#ifndef TENSOR_H
#define TENSOR_H

#include "Messages.h"
#include "VectorTypes.h"
#include "Constants.h"

#include <cmath>

namespace libRSF
{
  template<int Dim, int Size>
  class Tensor
  {
    public:
      Tensor() : Size_(std::pow(Size_, Dim))
      {
        Grid_.resize(Size_);
        Grid_.fill(NAN_DOUBLE);
      }
      ~Tensor() = default;

      template<typename... IndicesTemplate>
      double get(IndicesTemplate... Indices) const
      {
        /** check parameter number */
        static_assert(sizeof...(IndicesTemplate) == Dim, "Number of provided indices have to be equal to the number of dimensions!");

        /** get value */
        return Grid_(Index_(Indices...));
      }

      double get(const VectorStatic<Dim> Indices) const
      {
        return Grid_(Index_(Indices));
      }

      void set(const double Value, const VectorStatic<Dim> Indices)
      {
        Grid_(Index_(Indices)) = Value;
      }

      template<typename... IndicesTemplate>
      void set(const double Value, IndicesTemplate... Indices)
      {
        /** check parameter number */
        static_assert(sizeof...(IndicesTemplate) == Dim, "Number of provided indices have to be equal to the number of dimensions!");

        Grid_(Index_(Indices...)) = Value;
      }

      void print() const
      {
        PRINT_LOGGING(Grid_.transpose());
      }

    private:

      /** convert index for parameter pack*/
      template<typename... IndicesTemplate>
      int Index_(const int Index, IndicesTemplate... Indices) const
      {
        return Index + Index_(Indices...) * Size_;
      }
      [[nodiscard]] int Index_(const int Index) const
      {
        if(Index >= Size_)
        {
          PRINT_ERROR("Index exceed tensor size!");
        }

        return Index;
      }

      /** convert index from vector */
      int Index_(const VectorStatic<Dim> Indices) const
      {
        int Index = 0;
        for (int n = Dim-1; n >=0 ; n--)
        {
          Index *= Size_;
          Index += Indices(n);
        }

        if(Index >= Size_)
        {
          PRINT_ERROR("Index exceed tensor size!");
        }

        return Index;
      }

      /** storage for grid points */
      Vector Grid_;
      const int Size_;
  };

}

#endif // TENSOR_H
