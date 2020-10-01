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
 * @file Data.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Base class that is able to represent probabilistic data.
 * @copyright GNU Public License.
 *
 */

#ifndef DATA_H
#define DATA_H

#include "Messages.h"
#include "VectorMath.h"

#include <cstdio>
#include <string>

namespace libRSF
{
  template<typename TypeEnum, typename ElementEnum>
  class DataConfig
  {
    public:

      /** define types that store the configuration */
      typedef std::vector<std::pair<ElementEnum, size_t>> ConfigType;
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

  template<typename TypeEnum, typename ElementEnum>
  class Data
  {
    typedef DataConfig<TypeEnum, ElementEnum> ConfigType;

    public:
      Data() = default;
      virtual ~Data() = default;

      /** get properties */
      TypeEnum getType() const
      {
        return _Type;
      }

      std::string getName() const
      {
        return _Name;
      }

      /** get elements */
      Vector getValue(const ElementEnum Element) const
      {
        return _Data.at(Element);
      }

      Vector getMean() const
      {
        return getValue(ElementEnum::Mean);
      }

      double getTimestamp() const
      {
        return getValue(ElementEnum::Timestamp).operator()(0);
      }

      /** get pointers */
      double* getDataPointer(const ElementEnum Element)
      {
        return _Data.at(Element).data();
      }

      double* getMeanPointer()
      {
        return getDataPointer(ElementEnum::Mean);
      }

      double const * getMeanPointerConst()
      {
        return getDataPointer(ElementEnum::Mean);
      }

      /** set elements */
      void setValue(const ElementEnum Element, const Vector Value)
      {
        _Data.at(Element) = Value;
      }

      void setValueScalar(const ElementEnum Element, const double Value)
      {
        _Data.at(Element).fill(Value);
      }

      void setMean(const Vector Value)
      {
        setValue(ElementEnum::Mean, Value);
      }

      void setTimestamp(const double Timestamp)
      {
        Vector1 TS;
        TS << Timestamp;
        setValue(ElementEnum::Timestamp, TS);
      }

      Vector getCovariance() const
      {
        return this->getValue(ElementEnum::Covariance);
      }

      Matrix getCovarianceMatrix() const
      {
        Vector CovVect = this->getCovariance();
        MatrixRef<double, Dynamic, Dynamic> Cov(CovVect.data(), sqrt(CovVect.size()), sqrt(CovVect.size()));
        return Cov;
      }

      void setCovariance(const Vector Cov)
      {
        setValue(ElementEnum::Covariance, Cov);
      }

      /** change element size */
      void resizeElement(const ElementEnum Element, const size_t Size)
      {
        _Data.at(Element).resize(Size);
      }

      /** generate pretty output strings */
      std::string getValueString() const
      {
        std::string Out;

        for(const auto &Element : _Config->getConfig(_Type))
        {
          for(Index nElement = 0; nElement < _Data.at(Element.first).size(); nElement++)
          {
            std::ostringstream Stream;
            Stream.precision(8);
            Stream << std::scientific << _Data.at(Element.first).operator[](nElement);

            Out.append(Stream.str());
            Out.append(" ");
          }
        }

        return Out;
      }

      std::string getNameValueString() const
      {
        std::string Out;
        Out.append(_Name);
        Out.append(": ");

        for(auto const &Element : _Data)
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
        if(_Config->checkType(Type))
        {
          _Type = Type;
          _Name = _Config->getName(Type);

          for(const auto &Element : _Config->getConfig(Type))
          {
            _Data.emplace(Element.first, Vector(Element.second));
            _Data.at(Element.first).fill(0.0);
          }

          _Data[ElementEnum::Timestamp].operator[](0) = Timestamp;
        }
        else
        {
          PRINT_ERROR("Type does not exist: ", Type);
        }
      }

      void constructFromString(std::string Input)
      {
        /** read type from string */
        auto Split = Input.find_first_of(' ');
        std::string Name = Input.substr(0, Split);

        /** choose config according to type */
        if(_Config->checkName(Name))
        {
          constructEmpty(_Config->getType(Name));
          parseSubstring(Input.substr(Split));
        }
        else
        {
          PRINT_ERROR("Type does not exist: ", Name);
        }
      }

    /** pointer to the the list of configurations */
    const ConfigType * _Config;

    private:
      /** parse an ASCII input string */
      std::string parseSubstring(std::string Input)
      {
        size_t StringEnd = 0;
        size_t InputStringEnd = 0;

        for(const auto &Element : _Config->getConfig(_Type))
        {
          for(Index nElement = 0; nElement < _Data.at(Element.first).size(); nElement++)
          {
            _Data.at(Element.first).operator[](nElement) = std::stod(Input.substr(InputStringEnd), &StringEnd);
            InputStringEnd += StringEnd;
          }
        }

        return Input.substr(InputStringEnd);
      }

      /** identifying string */
      std::string _Name;

      /** internal type */
      TypeEnum _Type;

      /** where the data is stored */
      std::map<ElementEnum, Vector> _Data;
  };
}

#endif // DATA_H
