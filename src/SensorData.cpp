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

#include "SensorData.h"

namespace libRSF
{
  const SensorConfig Sensors;

  SensorConfig::SensorConfig()
  {
    /** list of known sensors, add new ones at the end*/
    DataConfigElementType Odom2DiffConfig;
    Odom2DiffConfig._Type = SensorType::Odom2Diff;
    Odom2DiffConfig._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    Odom2DiffConfig._Elements.emplace_back(make_pair(SensorElement::Mean, 3));
    Odom2DiffConfig._Elements.emplace_back(make_pair(SensorElement::WheelBase, 1));
    Odom2DiffConfig._Elements.emplace_back(make_pair(SensorElement::StdDev, 3));
    Odom2DiffConfig._Name = "Differential Odometry 2D";
    _Config.emplace(Odom2DiffConfig._Type, Odom2DiffConfig);
    _Identifier.emplace("odom2diff", Odom2DiffConfig._Type);

    DataConfigElementType Odom3Config;
    Odom3Config._Type = SensorType::Odom3;
    Odom3Config._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    Odom3Config._Elements.emplace_back(make_pair(SensorElement::Mean, 6));
    Odom3Config._Elements.emplace_back(make_pair(SensorElement::StdDev, 6));
    Odom3Config._Name = "Odometry 3D";
    _Config.emplace(Odom3Config._Type, Odom3Config);
    _Identifier.emplace("odom3", Odom3Config._Type);

    DataConfigElementType Range2Config;
    Range2Config._Type = SensorType::Range2;
    Range2Config._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    Range2Config._Elements.emplace_back(make_pair(SensorElement::Mean, 1));
    Range2Config._Elements.emplace_back(make_pair(SensorElement::StdDev, 1));
    Range2Config._Elements.emplace_back(make_pair(SensorElement::SatPos, 2));
    Range2Config._Elements.emplace_back(make_pair(SensorElement::SatID, 1));
    Range2Config._Name = "Range 2D";
    _Config.emplace(Range2Config._Type, Range2Config);
    _Identifier.emplace("range2", Range2Config._Type);

    DataConfigElementType Range3Config;
    Range3Config._Type = SensorType::Range3;
    Range3Config._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    Range3Config._Elements.emplace_back(make_pair(SensorElement::Mean, 1));
    Range3Config._Elements.emplace_back(make_pair(SensorElement::StdDev, 1));
    Range3Config._Elements.emplace_back(make_pair(SensorElement::SatPos, 3));
    Range3Config._Elements.emplace_back(make_pair(SensorElement::SatID, 1));
    Range3Config._Elements.emplace_back(make_pair(SensorElement::SatElevation, 1));
    Range3Config._Elements.emplace_back(make_pair(SensorElement::SNR, 1));
    Range3Config._Name = "Range 3D";
    _Config.emplace(Range3Config._Type, Range3Config);
    _Identifier.emplace("range3", Range3Config._Type);

    DataConfigElementType Pose2Config;
    Pose2Config._Type = SensorType::GT2;
    Pose2Config._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    Pose2Config._Elements.emplace_back(make_pair(SensorElement::Mean, 2));
    Pose2Config._Name = "Ground Truth 2D";
    _Config.emplace(Pose2Config._Type, Pose2Config);
    _Identifier.emplace("gt2", Pose2Config._Type);

    DataConfigElementType Pose3Config;
    Pose3Config._Type = SensorType::GT3;
    Pose3Config._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    Pose3Config._Elements.emplace_back(make_pair(SensorElement::Mean, 3));
    Pose3Config._Name = "Ground Truth 3D";
    _Config.emplace(Pose3Config._Type, Pose3Config);
    _Identifier.emplace("gt3", Pose3Config._Type);

    DataConfigElementType DeltaTimeConfig;
    DeltaTimeConfig._Type = SensorType::DeltaTime;
    DeltaTimeConfig._Elements.emplace_back(make_pair(SensorElement::Timestamp, 1));
    DeltaTimeConfig._Elements.emplace_back(make_pair(SensorElement::Mean, 1));
    DeltaTimeConfig._Name = "Delta Time";
    _Config.emplace(DeltaTimeConfig._Type, DeltaTimeConfig);
    _Identifier.emplace("deltat", DeltaTimeConfig._Type);

  }

  SensorData::SensorData()
  {
    _TypeDictionary = &Sensors;
  }

  SensorData::SensorData(string Input)
  {
    _TypeDictionary = &Sensors;
    constructFromString(Input);
  }

  SensorData::SensorData(SensorType Type, double Timestamp)
  {
    _TypeDictionary = &Sensors;
    constructEmpty(Type, Timestamp);
  }
}
