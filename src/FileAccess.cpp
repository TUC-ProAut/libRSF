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

#include "FileAccess.h"


namespace libRSF
{

  void ReadDataFromFile(const string& Filename,
                        SensorDataSet& SensorData)
  {
    string Buffer;
    std::ifstream File;

    File.open(Filename);
    std::getline(File, Buffer);

    while(Buffer.length() > 0)
    {
      SensorData.addElement(Data(Buffer));
      std::getline(File, Buffer);
    }

    File.close();
  }

  void WriteDataToFile(const string& Filename,
                       const string& DataName,
                       const StateDataSet& SensorData,
                       const bool Append)
  {
    double Timestamp;

    if(!SensorData.getTimeFirst(DataName,Timestamp))
    {
      return;
    }

    std::ofstream File;

    if(Append)
    {
      File.open(Filename, std::ios::out | std::ios::app);
    }
    else
    {
      File.open(Filename, std::ios::out | std::ios::trunc);
    }

    do
    {
      int NumberOfStates = SensorData.countElement(DataName,Timestamp);
      for (int nState = 0; nState < NumberOfStates; ++nState)
      {
        Data State;
        SensorData.getElement(DataName, Timestamp, nState, State);
        File << State.getName() << ' ' << State.getValueString() << std::endl;
      }
    }
    while(SensorData.getTimeNext(DataName, Timestamp, Timestamp));

    File.close();
  }
}
