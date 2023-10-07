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
 * @file DataGeneric.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Base class, that is able to represent all kinds of structured data.
 * @copyright GNU Public License.
 *
 */

#ifndef DATAGENERIC_H
#define DATAGENERIC_H

#include "Messages.h"
#include "VectorTypes.h"
#include "DataConfig.h"

#include <cstdio>
#include <string>

namespace libRSF
{
  template<typename TypeEnum, typename ElementEnum>
  class DataGeneric
  {
    using ConfigType = DataConfig<TypeEnum, ElementEnum>;

    public:
      DataGeneric() = default;
      virtual ~DataGeneric() = default;

      /** get properties */
      TypeEnum getType() const
      {
        return Type_;
      }

      [[nodiscard]] std::string getName() const
      {
        return Name_;
      }

      /** get elements */
      Vector getValue(const ElementEnum Element) const
      {
        return Data_.at(Element);
      }

      /** get pointers */
      double* getDataPointer(const ElementEnum Element)
      {
        return Data_.at(Element).data();
      }

      /** set elements */
      void setValue(const ElementEnum Element, const Vector& Value)
      {
        Data_.at(Element) = Value;
      }

      void setValueScalar(const ElementEnum Element, const double Value)
      {
        Data_.at(Element).fill(Value);
      }

      /** check if element exists */
      bool checkElement(const ElementEnum Element) const
      {
        return (Data_.count(Element) > 0);
      }

      /** generate pretty output strings */
      [[nodiscard]] std::string getValueString() const
      {
        std::string Out;

        for(const auto &Element : Config->getConfig(Type_))
        {
          for(Index nElement = 0; nElement < Data_.at(Element.first).size(); nElement++)
          {
            std::ostringstream Stream;
            Stream.precision(8);
            Stream << std::scientific << Data_.at(Element.first).operator[](nElement);

            Out.append(Stream.str());
            Out.append(" ");
          }
        }

        return Out;
      }

      [[nodiscard]] std::string getNameValueString() const
      {
        std::string Out;
        Out.append(Name_);
        Out.append(": ");

        for(auto const &Element : Data_)
        {
          Out.append(" ");

          for(Index nElement = 0; nElement < Element.second.size(); nElement++)
          {
            Out.append(std::to_string(Element.second.operator[](nElement)));
            Out.append(" ");
          }
        }

        return Out;
      }

    protected:
      /** construct a specific data configuration */
      void constructEmpty(const TypeEnum Type, double Timestamp = 0.0)
      {
        if(Config->checkType(Type))
        {
          Type_ = Type;
          Name_ = Config->getName(Type);

          for(const auto &Element : Config->getConfig(Type))
          {
            Data_.emplace(Element.first, Vector(Element.second));
            Data_.at(Element.first).fill(0.0);
          }

          Data_[ElementEnum::Timestamp].operator[](0) = Timestamp;
        }
        else
        {
          PRINT_ERROR("Type does not exist: ", Type);
        }
      }

      void constructFromString(const std::string& Input)
      {
        /** read type from string */
        auto Split = Input.find_first_of(' ');
        std::string Name = Input.substr(0, Split);

        /** choose config according to type */
        if(Config->checkName(Name))
        {
          constructEmpty(Config->getType(Name));
          parseSubstring_(Input.substr(Split));
        }
        else
        {
          PRINT_ERROR("Type does not exist: ", Name);
        }
      }

    /** pointer to the the list of configurations */
    const ConfigType * Config;

    private:
      /** parse an ASCII input string */
      std::string parseSubstring_(const std::string& Input)
      {
        size_t StringEnd = 0;
        size_t InputStringEnd = 0;

        for(const auto &Element : Config->getConfig(Type_))
        {
          for(Index nElement = 0; nElement < Data_.at(Element.first).size(); nElement++)
          {
            Data_.at(Element.first).operator[](nElement) = std::stod(Input.substr(InputStringEnd), &StringEnd);
            InputStringEnd += StringEnd;
          }
        }

        return Input.substr(InputStringEnd);
      }

      /** identifying string */
      std::string Name_;

      /** internal type */
      TypeEnum Type_;

      /** where the data is stored */
      std::map<ElementEnum, Vector> Data_;
  };
}

#endif // DATAGENERIC_H
