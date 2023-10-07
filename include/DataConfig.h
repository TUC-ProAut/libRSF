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
      using ConfigType = std::vector<std::pair<ElementEnum, int>>;
      using InitType = struct
      {
        std::string Name;
        TypeEnum Type;
        ConfigType Elements;
      };
      using InitVect = std::vector<InitType>;

      /** disable the default constructor construction */
      DataConfig() = delete;

      /** enforce advanced initialization */
      explicit DataConfig(InitVect InitialConfig)
      {
        for (InitType &Init : InitialConfig)
        {
          TypeMap_.emplace(Init.Type, Init.Elements);
          NameTypeMap_.emplace(Init.Name, Init.Type);
          TypeNameMap_.emplace(Init.Type, Init.Name);
        }
      }

      /** destruction */
      virtual ~DataConfig() = default;

      /** query string */
      std::string getName(TypeEnum Type) const
      {
        return TypeNameMap_.at(Type);
      }

      TypeEnum getType(std::string Name) const
      {
        return NameTypeMap_.at(Name);
      }

      /** check type or string */
      [[nodiscard]] bool checkName(std::string Name) const
      {
        return (NameTypeMap_.count(Name) > 0);
      }

      bool checkType(TypeEnum Type) const
      {
        return (TypeNameMap_.count(Type) > 0);
      }

      /** query config */
      const ConfigType &getConfig(std::string ID) const
      {
        return TypeMap_.at(NameTypeMap_.at(ID));
      }

      const ConfigType &getConfig(TypeEnum Type) const
      {
        return TypeMap_.at(Type);
      }

    private:
      std::map<std::string, TypeEnum> NameTypeMap_;
      std::map<TypeEnum, std::string> TypeNameMap_;
      std::map<TypeEnum, ConfigType> TypeMap_;
  };
}

#endif // DATACONFIG_H
