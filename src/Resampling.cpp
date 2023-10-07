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

#include "Resampling.h"

namespace libRSF
{
  std::vector<Data> SampleMeasurementsDown(const std::vector<Data> &Input, const double SampleTime)
  {
    std::vector<Data> Output;

    if(Input.empty())
    {
      PRINT_ERROR("There is no Measurement!");
      return Output;
    }

    double Time = Input.front().getTimestamp();
    double TimeNext = Time + SampleTime;
    double TimeMax = Input.back().getTimestamp();

    std::vector<Data> AveragingWindow;
    for(const Data &Sensor : Input)
    {
      /** group measurements in periods of sample time length*/
      AveragingWindow.push_back(Sensor);

      /** average when period is over*/
      if(Sensor.getTimestamp() >= TimeNext || Sensor.getTimestamp() == TimeMax)
      {
        /** average measurements */
        Output.push_back(AverageMeasurement(AveragingWindow));

        /** set timestamp to last used measurements */
        Output.back().setTimestamp(AveragingWindow.back().getTimestamp());

        /** set end of new window and clear old one */
        AveragingWindow.clear();
        TimeNext += SampleTime;
      }
    }

    return Output;
  }

  Data AverageMeasurement(const std::vector<Data> &Input)
  {
    Data Output;

    /** catch empty vectors */
    if(Input.empty())
    {
      PRINT_ERROR("There is no Measurement!");
      return Output;
    }

    /** return single element vectors directly */
    if(Input.size() == 1)
    {
      return Input.at(0);
    }

    /** create vectors */
    double Time = 0;
    Vector Mean (Input.front().getMean().size());
    Mean.setZero();
    Vector Info (Input.front().getCovarianceDiagonal().size());
    Info.setZero();
    for (const Data & Measurement : Input)
    {
      Time += Measurement.getTimestamp();
      Mean += Measurement.getMean();
      Info += Measurement.getCovarianceDiagonal().cwiseInverse();
    }
    Mean /= static_cast<double>(Input.size());
    Time /= static_cast<double>(Input.size());

    Output = Input.back();
    Output.setTimestamp(Time);
    Output.setMean(Mean);
    Output.setCovarianceDiagonal(Info.cwiseInverse());

    return Output;
  }

}
