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
 * @file SensorDataSet.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A class that stores multiple streams of measurements with timestamps.
 * @copyright GNU Public License.
 *
 */

#ifndef SENSORDATASET_H
#define SENSORDATASET_H

#include "DataSet.h"
#include "Data.h"

namespace libRSF
{

  class SensorDataSet : public DataSet<DataType, Data>
  {
    public:
      SensorDataSet() = default;
      ~SensorDataSet() override = default;

      /** add an element according to its internal type and timestamp*/
      void addElement(const Data& Element);

      using DataSet<DataType, Data>::addElement;
  };

  using MeasurementID = SensorDataSet::UniqueID;

  std::ostream& operator << (std::ostream& Os, const MeasurementID& ID);
}

#endif // SENSORDATASET_H
