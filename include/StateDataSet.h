/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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
 * @file StateDataSet.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A class that stores multiple streams of state variables with timestamps.
 * @copyright GNU Public License.
 *
 */

#ifndef STATEDATASET_H
#define STATEDATASET_H

#include "StateData.h"
#include "DataSet.h"

namespace libRSF
{
  class StateDataSet : public DataSet<std::string, StateData>
  {
    public:
      StateDataSet() {};
      ~StateDataSet() {};

      /** add an element according to its internal type and timestamp*/
      void addElement(StateData &Element);
      /** use external name */
      void addElement(std::string Name, StateData &Element);
      /** add an empty element*/
      void addElement(std::string Name, StateType Type, double Timestamp);

      using DataSet<std::string, StateData>::addElement;
  };

  typedef StateDataSet::UniqueID StateID;

  std::ostream& operator << (std::ostream& Os, const StateID& ID);
}

#endif // STATEDATASET_H
