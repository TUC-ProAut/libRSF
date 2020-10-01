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

#include "TimeMeasurement.h"

namespace libRSF
{
  Timer::Timer()
  {
    _Start = std::chrono::high_resolution_clock::now();
  }

  void Timer::reset()
  {
    _Start = std::chrono::high_resolution_clock::now();
  }

  double Timer::getMilliseconds()
  {
    /** catch current time */
    TimestampType End = std::chrono::high_resolution_clock::now();

    /** convert to a duration */
    double TimeDifference = std::chrono::duration_cast<std::chrono::nanoseconds>(End - _Start).count() / 1e6;

    /** convert to double */
    return TimeDifference;
  }

  double Timer::getSeconds()
  {
    /** catch current time */
    TimestampType End = std::chrono::high_resolution_clock::now();

    /** convert to a duration */
    double TimeDifference = std::chrono::duration_cast<std::chrono::nanoseconds>(End - _Start).count() / 1e9;

    /** convert to double */
    return TimeDifference;
  }

}
