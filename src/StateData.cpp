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

#include "StateData.h"

using std::string;
using std::map;
using std::pair;

namespace libRSF
{
  const StateConfig States;

  StateConfig::StateConfig()
  {
    /** list of known states, add new ones at the end*/
    DataConfigElementType Pose2Config;
    Pose2Config._Type = StateType::Pose2;
    Pose2Config._Elements.emplace_back(make_pair(StateElement::Timestamp, 1));
    Pose2Config._Elements.emplace_back(make_pair(StateElement::Mean, 2));
    Pose2Config._Elements.emplace_back(make_pair(StateElement::Covariance, 4));
    Pose2Config._Name = "Position 2D";
    _Config.emplace(Pose2Config._Type, Pose2Config);
    _Identifier.emplace("pos2", Pose2Config._Type);

    DataConfigElementType Pose3Config;
    Pose3Config._Type = StateType::Pose3;
    Pose3Config._Elements.emplace_back(make_pair(StateElement::Timestamp, 1));
    Pose3Config._Elements.emplace_back(make_pair(StateElement::Mean, 3));
    Pose3Config._Elements.emplace_back(make_pair(StateElement::Covariance, 9));
    Pose3Config._Name = "Position 3D";
    _Config.emplace(Pose3Config._Type, Pose3Config);
    _Identifier.emplace("pos3", Pose3Config._Type);

    DataConfigElementType AngleConfig;
    AngleConfig._Type = StateType::Angle;
    AngleConfig._Elements.emplace_back(make_pair(StateElement::Timestamp, 1));
    AngleConfig._Elements.emplace_back(make_pair(StateElement::Mean, 1));
    AngleConfig._Elements.emplace_back(make_pair(StateElement::Covariance, 1));
    AngleConfig._Name = "Angle";
    _Config.emplace(AngleConfig._Type, AngleConfig);
    _Identifier.emplace("angle", AngleConfig._Type);

    DataConfigElementType Value1Config;
    Value1Config._Type = StateType::Val1;
    Value1Config._Elements.emplace_back(make_pair(StateElement::Timestamp, 1));
    Value1Config._Elements.emplace_back(make_pair(StateElement::Mean, 1));
    Value1Config._Elements.emplace_back(make_pair(StateElement::Covariance, 1));
    Value1Config._Name = "Value 1D";
    _Config.emplace(Value1Config._Type, Value1Config);
    _Identifier.emplace("val1", Value1Config._Type);

    DataConfigElementType DeltaTimeConfig;
    DeltaTimeConfig._Type = StateType::Time;
    DeltaTimeConfig._Elements.emplace_back(make_pair(StateElement::Timestamp, 1));
    DeltaTimeConfig._Elements.emplace_back(make_pair(StateElement::Mean, 1));
    DeltaTimeConfig._Name = "Time";
    _Config.emplace(DeltaTimeConfig._Type, DeltaTimeConfig);
    _Identifier.emplace("time", DeltaTimeConfig._Type);
  }

  StateData::StateData()
  {
    _TypeDictionary = &States;
  }

  StateData::StateData(string Input)
  {
    _TypeDictionary = &States;
    constructFromString(Input);
  }

  StateData::StateData(StateType Type, double Timestamp)
  {
    _TypeDictionary = &States;
    constructEmpty(Type, Timestamp);
  }
}
