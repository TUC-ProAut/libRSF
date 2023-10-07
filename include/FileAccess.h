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
 * @file FileAccess.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Transfers data from files to DataSet objects and vice versa.
 * @copyright GNU Public License.
 *
 */

#ifndef FILEACCESS_H
#define FILEACCESS_H

#include "StateDataSet.h"
#include "SensorDataSet.h"

#include <ceres/ceres.h>

#include <cmath>

#include <cstdio>
#include <vector>
#include <random>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <memory>

using std::vector;
using std::string;
using std::ofstream;

namespace libRSF
{
  void ReadDataFromFile(const string& Filename,
                        SensorDataSet& Data);

  void WriteDataToFile(const string& Filename,
                       const string& DataName,
                       const StateDataSet& Data,
                       bool Append = false);
}

#endif // FILEACCESS_H
