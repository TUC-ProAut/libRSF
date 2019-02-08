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

#include <cstdio>
#include <string>
#include <ceres/ceres.h>

using std::string;
using std::map;
using std::multimap;
using std::pair;
using std::make_pair;
using std::set;
using std::vector;

namespace libRSF
{

  template<typename EnumType, typename EnumElement>
  struct DataConfig
  {
    typedef EnumType DataEnumType;
    typedef EnumElement ElementEnumType;

    typedef struct
    {
      vector<pair<ElementEnumType, size_t>> _Elements;
      DataEnumType _Type;
      string _Name;
    } DataConfigElementType;

    std::map<const DataEnumType, const DataConfigElementType> _Config;
    std::map<const string, const DataEnumType> _Identifier;
  };

  template<typename TypeConfig>
  class Data
  {
    public:
      Data() {};
      ~Data() {};

      typedef typename TypeConfig::DataConfigElementType ConfigType;
      typedef typename TypeConfig::ElementEnumType ElementType;
      typedef typename TypeConfig::DataEnumType TypeType;

      /** parse an ASCII input string */
      string parseSubstring(string Input)
      {
        size_t StringEnd = 0;
        size_t InputStringEnd = 0;

        for(const auto &Element : _Type->_Elements)
        {
          for(size_t nElement = 0; nElement < _Data.at(Element.first).size(); nElement++)
          {
            _Data.at(Element.first).operator[](nElement) = std::stod(Input.substr(InputStringEnd), &StringEnd);
            InputStringEnd += StringEnd;
          }
        }

        return Input.substr(InputStringEnd);
      }

      /** generate pretty output strings */
      string getValueString()
      {
        string Out;

        for(const auto &Element : _Type->_Elements)
        {
          for(size_t nElement = 0; nElement < _Data.at(Element.first).size(); nElement++)
          {
            Out.append(std::to_string(_Data.at(Element.first).operator[](nElement)));
            Out.append(" ");
          }
        }

        return Out;
      }

      /** get elements */
      ceres::Vector getValue(ElementType Element)
      {
        return _Data.at(Element);
      }

      ceres::Vector getMean()
      {
        return getValue(ElementType::Mean);
      }

      double getTimeStamp()
      {
        return getValue(ElementType::Timestamp).operator[](0);
      }

      TypeType getType()
      {
        return _Type->_Type;
      }

      string getName()
      {
        return _Type->_Name;
      }

      double* getDataPointer(ElementType Element)
      {
        return _Data[Element].data();
      }

      double* getMeanPointer()
      {
        return getDataPointer(ElementType::Mean);
      }

      /** set elements */
      void setValue(ElementType Element, ceres::Vector Value)
      {
        _Data[Element] = Value;
      }

      void setMean(ceres::Vector Value)
      {
        setValue(ElementType::Mean, Value);
      }

      void setTimestamp(double Timestamp)
      {
        ceres::Vector TS(1);
        TS << Timestamp;
        setValue(ElementType::Timestamp, TS);
      }

    protected:

      /** construct a specific data configuration */
      void constructEmpty(const ConfigType* Type)
      {
        _Type = Type;

        for(const auto &Element : _Type->_Elements)
        {
          _Data.emplace(Element.first, ceres::Vector(Element.second));
          _Data.at(Element.first).fill(0.0);
        }

        return;
      }

      void constructEmpty(TypeType Type, double Timestamp)
      {
        /** choose config according to type */
        constructEmpty(&(_TypeDictionary->_Config.at(Type)));
        _Data[ElementType::Timestamp].operator[](0) = Timestamp;
      }

      void constructFromString(string Input)
      {
        /** read type from string */
        auto Split = Input.find_first_of(' ');
        string Type = Input.substr(0, Split);

        /** choose config according to type */
        if(_TypeDictionary->_Identifier.count(Type) > 0)
        {
          constructEmpty(&(_TypeDictionary->_Config.at(_TypeDictionary->_Identifier.at(Type))));
          parseSubstring(Input.substr(Split));
        }
        else
        {
          std::cerr << "In String-Constructor of Data: Type does not exist: " << Type << std::endl;
        }
      }

      /** pointer to a list of all elements according to the type */
      const ConfigType* _Type;

      TypeConfig const* _TypeDictionary;

      /** where the data is stored */
      map<ElementType, ceres::Vector> _Data;
  };
}

#endif // DATA_H
