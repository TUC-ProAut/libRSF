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
 * @file FactorIDSet.h
 * @author Tim Pfeifer
 * @date 19.07.2019
 * @brief A class that stores multiple streams of factor IDs.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORIDSET_H
#define FACTORIDSET_H

#include "DataSet.h"
#include "Types.h"

#include <ceres/ceres.h>

namespace libRSF
{
  using CeresFactorID = ceres::ResidualBlockId;

  class FactorIDSet : public DataSet<FactorType, CeresFactorID>
  {
    public:
      FactorIDSet() = default;
      ~FactorIDSet() override = default;

      /** specialized hash for FactorType enum */
      struct HashGlobalID
      {
        size_t operator() (const UniqueID &Object) const
        {
          size_t H1 = std::hash<size_t>()(static_cast<size_t>(Object.ID));
          size_t H2 = std::hash<double>()(roundToTick(Object.Timestamp));
          size_t H3 = std::hash<size_t>()(Object.Number);

          return H1 ^ H2 ^ H3;
        }
      };

      using DataSet<FactorType, CeresFactorID>::addElement;
  };

  using FactorID = FactorIDSet::UniqueID;

  std::ostream& operator << (std::ostream& Os, const FactorID& ID);
}

#endif // FACTORIDSET_H
