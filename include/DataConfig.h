/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2021 Chair of Automation Technology / TU Chemnitz
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
 * @file DataConfig.h
 * @author Tim Pfeifer
 * @date 09.03.2021
 * @brief This holds the configuration for different types of structured data.
 * @copyright GNU Public License.
 *
 */

#ifndef DATACONFIG_H
#define DATACONFIG_H

#include <map>
#include <vector>
#include <string>

namespace libRSF
{
  template<typename TypeEnum, typename ElementEnum>
  class DataConfig
  {
    public:

      /** define types that store the configuration */
      typedef std::vector<std::pair<ElementEnum, int>> ConfigType;
      typedef struct
      {
        std::string _Name;
        TypeEnum _Type;
        ConfigType _Elements;
      } InitType;
      typedef std::vector<InitType> InitVect;

      /** disable the default constructor construction */
      DataConfig() = delete;

      /** enforce advanced initialization */
      explicit DataConfig(InitVect InitialConfig)
      {
        for (InitType &Init : InitialConfig)
        {
          _TypeMap.emplace(Init._Type, Init._Elements);
          _NameTypeMap.emplace(Init._Name, Init._Type);
          _TypeNameMap.emplace(Init._Type, Init._Name);
        }
      }

      /** destruction */
      virtual ~DataConfig() = default;

      /** query string */
      std::string getName(TypeEnum Type) const
      {
        return _TypeNameMap.at(Type);
      }

      TypeEnum getType(std::string Name) const
      {
        return _NameTypeMap.at(Name);
      }

      /** check type or string */
      bool checkName(std::string Name) const
      {
        return (_NameTypeMap.count(Name) > 0);
      }

      bool checkType(TypeEnum Type) const
      {
        return (_TypeNameMap.count(Type) > 0);
      }

      /** query config */
      const ConfigType &getConfig(std::string ID) const
      {
        return _TypeMap.at(_NameTypeMap.at(ID));
      }

      const ConfigType &getConfig(TypeEnum Type) const
      {
        return _TypeMap.at(Type);
      }

    private:
      std::map<std::string, TypeEnum> _NameTypeMap;
      std::map<TypeEnum, std::string> _TypeNameMap;
      std::map<TypeEnum, ConfigType> _TypeMap;
  };
}

#endif // DATACONFIG_H
