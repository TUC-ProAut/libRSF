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
 * @file StateData.h
 * @author Tim Pfeifer
 * @date 21.11.2016
 * @brief Class which has the capability to represent generic state data.
 * @copyright GNU Public License.
 *
 */

#ifndef STATEDATA_H
#define STATEDATA_H

#include <ceres/ceres.h>

#include "Data.h"

namespace libRSF
{
  enum class StateType {Pose2, Pose3, Val1, Time, Angle};
  enum class StateElement {Timestamp, Mean, Covariance, GMM_StdDev, GMM_Weight, TF, Other};

  struct StateConfig : public DataConfig<StateType, StateElement>
  {
    StateConfig();
  };

  /** global object that hold all sensor configs */
  extern const StateConfig States;

  class StateData: public Data<StateConfig>
  {
    public:
      StateData();
      explicit StateData(string Input);
      StateData(StateType Type, double Timestamp);
      ~StateData() {};
  };
}

#endif // STATEDATA_H
