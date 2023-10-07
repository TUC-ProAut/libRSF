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
 * @file TimeMeasurement.h
 * @author Tim Pfeifer
 * @date 11.12.2019
 * @brief Helper function to compute the time between two events.
 * @copyright GNU Public License.
 *
 */

#ifndef TIMEMEASUREMENT_H
#define TIMEMEASUREMENT_H

#include "Messages.h"

#include <chrono>

namespace libRSF
{

  class Timer
  {
  public:
      Timer();
      ~Timer() = default;

      void reset();
      double getSeconds();
      double getMilliseconds();

  private:
    using TimestampType = std::chrono::high_resolution_clock::time_point;
    TimestampType Start_;
  };


}

#endif // TIMEMEASUREMENT_H
