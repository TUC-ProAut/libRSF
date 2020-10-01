/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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

#include "Types.h"


namespace libRSF
{
  std::ostream &operator << (std::ostream& Os, const SensorType& Type)
  {
    Os <<  static_cast<int>(Type);
    return Os;
  }

  std::ostream &operator << (std::ostream& Os, const StateType& Type)
  {
    Os <<  static_cast<int>(Type);
    return Os;
  }

  std::ostream& operator << (std::ostream& Os, const FactorType& Type)
  {
    for (const auto &Entry : FactorTypeDict)
    {
      if(Entry.second == Type)
      {
        Os << Entry.first;
      }
    }
    return Os;
  }

  std::ostream& operator << (std::ostream& Os, const ErrorModelType& Type)
  {
    for (const auto &Entry : ErrorModelTypeDict)
    {
      if(Entry.second == Type)
      {
        Os << Entry.first;
      }
    }
    return Os;
  }

  std::ostream &operator << (std::ostream& Os, const SolutionType& Type)
  {
    /** print numeric value */
    Os <<  static_cast<unsigned int>(Type);
    return Os;
  }
}
