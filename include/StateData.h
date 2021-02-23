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
 * @file StateData.h
 * @author Tim Pfeifer
 * @date 21.11.2016
 * @brief Class which has the capability to represent generic state data.
 * @copyright GNU Public License.
 *
 */

#ifndef STATEDATA_H
#define STATEDATA_H

#include "Data.h"
#include "Types.h"

namespace libRSF
{
  typedef DataConfig<StateType, StateElement> StateConfig;

  /** global object that hold all state configs */
  extern const StateConfig States;

  class StateData: public Data<StateType, StateElement>
  {
    public:
      StateData();
      explicit StateData(std::string Input);
      StateData(StateType Type, double Timestamp);

      virtual ~StateData() = default;

      Matrix getCovarianceMatrix()
      {
        Vector CovVect = this->getCovariance();
        switch (CovVect.size())
        {
        case 1:
          return Matrix11(CovVect.data());
            break;

        case 4:
          return Matrix22(CovVect.data());
            break;

        case 9:
          return Matrix33(CovVect.data());
            break;

        default:
          PRINT_WARNING("Covariance matrix with ", CovVect.size(), " elements is not expected! Return empty matrix!");
          return Matrix11::Zero();
            break;
        }
      }
  };
}

#endif // STATEDATA_H
