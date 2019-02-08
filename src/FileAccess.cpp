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

#include "FileAccess.h"


namespace libRSF
{

  void ReadDataFromFile(string Filename,
                        SensorDataSet &Data)
  {
    string Buffer;
    std::ifstream File;

    File.open(Filename);
    std::getline(File, Buffer);

    while(Buffer.length() > 0)
    {
      Data.addElement(SensorData(Buffer));
      std::getline(File, Buffer);
    }

    File.close();
  }

  void WriteDataToFile(string Filename,
                       string DataName,
                       StateDataSet &Data)
  {
    double Timestamp;
    size_t NumberOfStates;

    if(!Data.getFirstTimestamp(DataName, Timestamp))
      return;

    std::ofstream File;
    File.open(Filename, std::ios::out | std::ios::trunc);

    do
    {
      NumberOfStates = Data.countElement(DataName, Timestamp);

      for(size_t nState = 0; nState < NumberOfStates; ++nState)
      {
        File << Data.getElement(DataName, Timestamp, nState).getValueString();
        File << std::endl;
      }
    }
    while(Data.getNextTimestamp(DataName, Timestamp, Timestamp));

    File.close();
  }
}
