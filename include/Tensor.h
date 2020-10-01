/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
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
* @file VectorTypes.h
* @author Tim Pfeifer
* @date 19.05.2020
* @brief Derived types for vectors, matrices and 2D/3D rotations.
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
      Tensor() : _Size(std::pow(Size, Dim))
      {
        _Grid.resize(_Size);
        _Grid.fill(NAN_DOUBLE);
      }
      ~Tensor() = default;

      template<typename... IndicesTemplate>
      double get(IndicesTemplate... Indices) const
      {
        /** check parameter number */
        static_assert(sizeof...(IndicesTemplate) == Dim, "Number of provided indices have to be equal to the number of dimensions!");

        /** get value */
        return _Grid(_Index(Indices...));
      }

      double get(const VectorStatic<Dim> Indices) const
      {
        return _Grid(_Index(Indices));
      }

      void set(const double Value, const VectorStatic<Dim> Indices)
      {
        _Grid(_Index(Indices)) = Value;
      }

      template<typename... IndicesTemplate>
      void set(const double Value, IndicesTemplate... Indices)
      {
        /** check parameter number */
        static_assert(sizeof...(IndicesTemplate) == Dim, "Number of provided indices have to be equal to the number of dimensions!");

        /** get Index */
        int Index = _Index(Indices...);

        _Grid(_Index(Indices...)) = Value;
      }

      void print() const
      {
        PRINT_LOGGING(_Grid.transpose());
      }

    private:

      /** convert index for parameter pack*/
      template<typename... IndicesTemplate>
      int _Index(const int Index, IndicesTemplate... Indices) const
      {
        return Index + _Index(Indices...) * Size;
      }
      int _Index(const int Index) const
      {
        if(Index >= _Size)
        {
          PRINT_ERROR("Index exceed tensor size!");
        }

        return Index;
      }

      /** convert index from vector */
      int _Index(const VectorStatic<Dim> Indices) const
      {
        int Index = 0;
        for (int n = Dim-1; n >=0 ; n--)
        {
          Index *= Size;
          Index += Indices(n);
        }

        if(Index >= _Size)
        {
          PRINT_ERROR("Index exceed tensor size!");
        }

        return Index;
      }

      /** storage for grid points */
      Vector _Grid;
      const int _Size;
  };

}

#endif // TENSOR_H
