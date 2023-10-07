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
 * @file Hash.h
 * @author Tim Pfeifer
 * @date 24.07.2019
 * @brief Hashing functions for custom data types.
 * @copyright GNU Public License.
 *
 */

#ifndef HASH_H
#define HASH_H

#include "FactorIDSet.h"
#include "StateDataSet.h"
#include "SensorDataSet.h"

/** has to be in the std namespace! */
namespace std
{
  using std::size_t;
  using std::hash;
  using std::string;

  size_t CombineHash(size_t H1, size_t H2, size_t H3);

  template <>
  struct hash<libRSF::FactorType>
  {
    size_t operator()(const libRSF::FactorType& Object) const
    {
      return hash<size_t>()(static_cast<size_t>(Object));
    }
  };

  template<>
  struct hash<libRSF::FactorID>
  {
    size_t operator()(const libRSF::FactorID& Object) const
    {
      return CombineHash(hash<libRSF::FactorType>()(Object.ID),
                         hash<double>()(Object.Timestamp),
                         hash<size_t>()(Object.Number));
    }
  };

  template<>
  struct hash<libRSF::StateID>
  {
    size_t operator()(const libRSF::StateID& Object) const
    {
      return CombineHash(hash<string>()(Object.ID),
                         hash<double>()(Object.Timestamp),
                         hash<size_t>()(Object.Number));
    }
  };

}


#endif // HASH_H
