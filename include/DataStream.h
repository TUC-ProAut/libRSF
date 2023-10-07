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
 * @file DataStream.h
 * @author Tim Pfeifer
 * @date 08.03.2021
 * @brief Represents a stream of data objects in time. Keys are rounded to a fixed tick interval.
 * @copyright GNU Public License.
 *
 */

#ifndef DATASTREAM_H
#define DATASTREAM_H

#include <map>
#include <cmath>

namespace libRSF
{
  /** round the timestamp to ticks-precision */
  double roundToTick(double Time);

  /** stream of objects with ticked timestamps */
  template<typename ObjectType>
  class DataStream : private std::multimap<double, ObjectType>
  {
    public:
      DataStream() = default;
      virtual ~DataStream() = default;

      /** define base class */
      using BaseClass = typename std::multimap<double, ObjectType>;

      /** use the types of the base class */
      using iterator = typename BaseClass::iterator;
      using const_iterator = typename BaseClass::const_iterator;
      using size_type = typename BaseClass::size_type;

      /** implement all functions with a fixed tick size for the double keys */
      iterator find (const double &Time)
      {
        return BaseClass::find(roundToTick(Time));
      }

      const_iterator find (const double &Time) const
      {
        return BaseClass::find(roundToTick(Time));
      }

      size_type count (const double &Time) const
      {
        return BaseClass::count(roundToTick(Time));
      }

      iterator lower_bound (const double &Time)
      {
        return BaseClass::lower_bound(roundToTick(Time));
      }

      const_iterator lower_bound (const double &Time) const
      {
        return BaseClass::lower_bound(roundToTick(Time));
      }

      iterator upper_bound (const double &Time)
      {
        return BaseClass::upper_bound(roundToTick(Time));
      }

      const_iterator upper_bound (const double &Time) const
      {
        return BaseClass::upper_bound(roundToTick(Time));
      }

      std::pair<iterator,iterator> equal_range (const double &Time)
      {
        return BaseClass::equal_range(roundToTick(Time));
      }

      std::pair<const_iterator,const_iterator> equal_range (const double &Time) const
      {
        return BaseClass::equal_range(roundToTick(Time));
      }

      iterator emplace (const double &Time, const ObjectType &Object)
      {
        return BaseClass::emplace(roundToTick(Time), Object);
      }

      /** expose other functions */
      using BaseClass::erase;
      using BaseClass::begin;
      using BaseClass::end;
      using BaseClass::size;
      using BaseClass::empty;
  };
}

#endif // DATASTREAM_H
