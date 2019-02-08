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
 * @file ListInTime.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Base for Datasets that are able to store time dependent data streams.
 * @copyright GNU Public License.
 *
 */

#ifndef LIST_IN_TIME_H
#define LIST_IN_TIME_H

#include <ceres/ceres.h>

namespace libRSF
{
  /** one class as base for all lists of something in time */
  template<typename IdentifierType, typename ObjectType>
  class ListInTime
  {
    public:
      ListInTime() {};
      virtual ~ListInTime() {};

      typedef std::multimap<double, ObjectType> DataStream;

      /** add an element according to its ID and Timestamp*/
      void addElement(IdentifierType ID, double Timestamp, ObjectType Object)
      {
        if(_Data.count(ID) == 0)
        {
          DataStream TempStream;
          _Data.emplace(ID, TempStream);
        }

        _Data[ID].emplace(Timestamp, Object);
      }

      void removeElement(IdentifierType ID, double Timestamp)
      {
        if(checkElement(ID, Timestamp))
        {
          _Data.at(ID).erase(Timestamp);

          /** erase empty IDs */
          if(_Data.at(ID).empty())
          {
            _Data.erase(ID);
          }
        }
        else
        {
          std::cerr << "In removeElement: Element doesn't exist: " << ID << " " << Timestamp << " " << std::endl;
        }
      }

      /** check if an element exists */
      size_t countElement(IdentifierType ID, double Timestamp)
      {
        if(_Data.count(ID) == 0)
        {
          return 0;
        }
        else
        {
          return _Data[ID].count(Timestamp);
        }
      }

      bool checkElement(IdentifierType ID, double Timestamp, size_t Number = 0)
      {
        return (countElement(ID, Timestamp) > Number);
      }

      /** access elements */
      ObjectType &getElement(IdentifierType ID, double Timestamp, size_t Number = 0)
      {
        if(checkElement(ID, Timestamp, Number))
        {
          auto It = _Data[ID].find(Timestamp);
          std::advance(It, Number);
          return It->second;
        }
        else
        {
          std::cerr << "In getElement: Dataset element doesn't exist: " << ID << " " << Timestamp << " " << Number << std::endl;
        }
      }

      bool getElement(IdentifierType ID, double Timestamp, size_t Number, ObjectType &Element)
      {
        if(checkElement(ID, Timestamp, Number))
        {
          auto It = _Data[ID].find(Timestamp);
          std::advance(It, Number);
          Element = It->second;
          return true;
        }
        else
        {
          return false;
        }
      }

      bool getFirstTimestamp(IdentifierType ID, double &Timestamp)
      {
        if(_Data.count(ID) > 0)
        {
          Timestamp = _Data[ID].begin()->first;
          return true;
        }
        else
        {
          return false;
        }
      }

      bool getLastTimestamp(IdentifierType ID, double &Timestamp)
      {
        if(_Data.count(ID) > 0)
        {
          Timestamp = std::prev(_Data[ID].end())->first;
          return true;
        }
        else
        {
          return false;
        }
      }

      bool getNextTimestamp(IdentifierType ID, double Timestamp, double &NextTimeStamp)
      {
        size_t PrevElements = countElement(ID, Timestamp);

        if(PrevElements > 0)
        {
          auto It = _Data[ID].find(Timestamp);
          std::advance(It, PrevElements);

          if(It != _Data[ID].end())
          {
            NextTimeStamp = It->first;
          }
          else
          {
            return false;
          }

          return true;
        }
        else
        {
          return false;
        }
      }

      std::vector<ObjectType> getObjects(IdentifierType ID)
      {
        std::vector<ObjectType> Objects;

        for(auto it = _Data.at(ID).begin(); it != _Data.at(ID).end(); ++it)
        {
          Objects.push_back(it->second);
        }

        return Objects;
      }

      auto begin()
      {
        return _Data.begin();
      }

      auto end()
      {
        return _Data.end();
      }

    protected:
      std::map<IdentifierType, std::multimap<double, ObjectType>> _Data;
  };

}

#endif // LIST_IN_TIME_H
