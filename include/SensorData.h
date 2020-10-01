/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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
 * @file SensorData.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A class that stores one sensor measurement.
 * @copyright GNU Public License.
 *
 */

#ifndef SENSORDATA_H
#define SENSORDATA_H

#include "Data.h"
#include "Types.h"
#include "VectorMath.h"

namespace libRSF
{
  typedef DataConfig<SensorType, SensorElement> SensorConfig;

  /** global object that hold all sensor configs */
  extern const SensorConfig Sensors;

  class SensorData: public Data<SensorType, SensorElement>
  {
    public:
      SensorData();
      virtual ~SensorData() = default;

      explicit SensorData(std::string Input);
      SensorData(SensorType Type, double Timestamp);

      Vector getStdDev() const
      {
        Vector StdDev = getValue(SensorElement::Covariance);
        return StdDev.cwiseSqrt();
      }

      void setStdDev(Vector Value)
      {
        Vector Cov = Value.array().square();
        setValue(SensorElement::Covariance, Cov);
      }
  };
}

#endif // SENSORDATA_H
