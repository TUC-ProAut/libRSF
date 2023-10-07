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
 * @file DataSet.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Base for Datasets that are able to store time dependent data streams.
 * @copyright GNU Public License.
 *
 */

#ifndef Data_SET_H
#define Data_SET_H

#include "Constants.h"
#include "DataStream.h"
#include "Messages.h"

namespace libRSF
{
  /** one class as base for all lists of something in time */
  template <typename KeyType, typename ObjectType>
  class DataSet
  {
   public:
    DataSet() = default;
    virtual ~DataSet() = default;

    /** chronological list of arbitrary objects*/
    using ObjectStream = DataStream<ObjectType>;

    /** unique ID that identifies one object */
    struct UniqueID
    {
      UniqueID() = default;

      UniqueID(const KeyType &IDNew, const double TimestampNew, const int NumberNew = 0) : ID(IDNew), Timestamp(TimestampNew), Number(NumberNew)
      {
      }

      /** is required to compare keys */
      bool operator==(const UniqueID &Other) const
      {
        return ID == Other.ID && Timestamp == Other.Timestamp && Number == Other.Number;
      }

      KeyType ID;
      double Timestamp;
      int Number;
    };

    /** add an element according to its ID and Timestamp*/
    void addElement(const KeyType &ID, const double &Timestamp, const ObjectType &Object)
    {
      if (!this->checkID(ID))
      {
        ObjectStream TempStream;
        DataStreams.emplace(ID, TempStream);
      }

      DataStreams.at(ID).emplace(Timestamp, Object);
    }

    void removeElement(const KeyType &ID, const double Timestamp, const int Number)
    {
      if (this->checkElement(ID, Timestamp, Number))
      {
        auto It = DataStreams.at(ID).lower_bound(Timestamp);
        std::advance(It, Number);
        DataStreams.at(ID).erase(It);

        /** erase empty IDs */
        if (DataStreams.at(ID).empty())
        {
          DataStreams.erase(ID);
        }
      }
      else
      {
        PRINT_ERROR("Element doesn't exist at: ", Timestamp, " Type: ", ID, " Number: ", Number);
      }
    }

    void removeElement(const KeyType &ID, const double Timestamp)
    {
      if (checkElement(ID, Timestamp))
      {
        DataStreams.at(ID).erase(Timestamp);

        /** erase empty IDs */
        if (DataStreams.at(ID).empty())
        {
          DataStreams.erase(ID);
        }
      }
      else
      {
        PRINT_ERROR("Element doesn't exist at: ", Timestamp, " Type: ", ID);
      }
    }

    void clear()
    {
      DataStreams.clear();
    }

    /** check if an element exists */
    int countElement(const KeyType &ID, const double Timestamp) const
    {
      if (!this->checkID(ID))
      {
        return 0;
      }

      return DataStreams.at(ID).count(Timestamp);
    }

    int countElements(const KeyType &ID) const
    {
      if (!this->checkID(ID))
      {
        return 0;
      }

      return DataStreams.at(ID).size();
    }

    bool checkElement(const KeyType &ID, const double Timestamp, const int Number = 0) const
    {
      return (countElement(ID, Timestamp) > Number);
    }

    bool checkID(const KeyType &ID) const
    {
      return (DataStreams.find(ID) != DataStreams.end());
    }

    /** check if any element is stored */
    bool empty()
    {
      return DataStreams.empty();
    }

    /** access elements */
    ObjectType &getElement(const KeyType &ID, const double Timestamp, const int Number = 0)
    {
      if (checkElement(ID, Timestamp, Number))
      {
        auto It = DataStreams.at(ID).lower_bound(Timestamp);
        std::advance(It, Number);
        return It->second;
      }

      PRINT_ERROR("Element doesn't exist at: ", Timestamp, " Type: ", ID, " Number: ", Number);
      return this->NullObject;
    }

    bool getElement(const KeyType &ID, const double Timestamp, const int Number, ObjectType &Element) const
    {
      if (checkElement(ID, Timestamp, Number))
      {
        auto It = DataStreams.at(ID).lower_bound(Timestamp);
        std::advance(It, Number);
        Element = It->second;
        return true;
      }

      return false;
    }

    bool setElement(const KeyType &ID, const double Timestamp, const int Number, ObjectType &Element)
    {
      if (checkElement(ID, Timestamp, Number))
      {
        auto It = DataStreams.at(ID).lower_bound(Timestamp);
        std::advance(It, Number);
        It->second = Element;
        return true;
      }

      PRINT_ERROR("Element doesn't exist at: ", Timestamp, " Type: ", ID, " Number: ", Number);
      return false;
    }

    bool getTimeFirst(const KeyType &ID, double &Timestamp) const
    {
      if (this->checkID(ID))
      {
        Timestamp = DataStreams.at(ID).begin()->first;
        return true;
      }

      return false;
    }

    bool getTimeFirstOverall(double &Timestamp) const
    {
      if (DataStreams.empty())
      {
        PRINT_ERROR("Empty list!");
        return false;
      }

      double TimeFirst = Timestamp;
      Timestamp = NAN_DOUBLE;
      std::vector<KeyType> IDs = this->getKeysAll();
      for (const KeyType &ID : IDs)
      {
        this->getTimeFirst(ID, TimeFirst);
        if (std::isnan(Timestamp) || TimeFirst < Timestamp)
        {
          Timestamp = TimeFirst;
        }
      }

      return true;
    }

    bool getTimeLast(const KeyType &ID, double &Timestamp) const
    {
      if (this->checkID(ID))
      {
        Timestamp = std::prev(DataStreams.at(ID).end())->first;
        return true;
      }

      return false;
    }

    bool getTimeNext(const KeyType &ID, const double Timestamp, double &NextTimeStamp) const
    {
      if (this->checkElement(ID, Timestamp))
      {
        auto It = DataStreams.at(ID).upper_bound(Timestamp);

        if (It != DataStreams.at(ID).end())
        {
          NextTimeStamp = It->first;
        }
        else
        {
          return false;
        }
        return true;
      }

      PRINT_ERROR("Key does not exist: ", ID);
      return false;
    }

    bool getTimePrev(const KeyType &ID, const double Timestamp, double &PrevTimeStamp) const
    {
      /** check if element exists */
      if(!this->checkElement(ID, Timestamp))
      {
        PRINT_ERROR("Previous element does not exist: ", ID, " at: ", Timestamp);
      }

      /** re-use function*/
      return this->getTimeBelow(ID, Timestamp, PrevTimeStamp);
    }

    bool getTimeAboveOrEqual(const KeyType &ID, const double TimeIn, double &TimeOut) const
    {
      if (!this->checkID(ID))
      {
        PRINT_ERROR("Key does not exist: ", ID);
        return false;
      }

      auto It = DataStreams.at(ID).lower_bound(TimeIn);

      if (It != DataStreams.at(ID).end())
      {
        TimeOut = It->first;
        return true;
      }

      return false;
    }

    bool getTimeAbove(const KeyType &ID, const double Timestamp, double &NextTimeStamp) const
    {
      if (!this->checkID(ID))
      {
        PRINT_ERROR("Key does not exist: ", ID);
        return false;
      }

      const auto It = DataStreams.at(ID).upper_bound(Timestamp);

      if (It != DataStreams.at(ID).end())
      {
        NextTimeStamp = It->first;
        return true;
      }

      return false;
    }

    bool getTimeBelow(const KeyType &ID, const double TimeIn, double &TimeOut) const
    {
      if (!this->checkID(ID))
      {
        PRINT_ERROR("Key does not exist: ", ID);
        return false;
      }

      const auto It = DataStreams.at(ID).lower_bound(TimeIn);

      /** case 1: there is no element below */
      if (It == DataStreams.at(ID).begin())
      {
        return false;
      }

      /** case 2: there is a element*/
      TimeOut = std::prev(It)->first;
      return true;
    }

    bool getTimeBelowOrEqual(const KeyType &ID, const double TimeIn, double &TimeOut) const
    {
      if (!this->checkID(ID))
      {
        PRINT_ERROR("Key does not exist: ", ID);
        return false;
      }

      /** catch equal case at first */
      if (DataStreams.at(ID).count(TimeIn) > 0)
      {
        const auto It = DataStreams.at(ID).lower_bound(TimeIn);
        TimeOut = It->first;
        return true; /**< equal */
      }

      return getTimeBelow(ID, TimeIn, TimeOut); /**< below */
    }

    bool getTimeCloseTo(const KeyType &ID, const double Timestamp, double &TimestampClose) const
    {
      /** catch non-existing key */
      if (!this->checkID(ID))
      {
        PRINT_ERROR("Key does not exist: ", ID);
        return false;
      }
      /** catch empty container */
      if (this->countElements(ID) == 0)
      {
        PRINT_ERROR("Container is empty for key: ", ID);
        return false;
      }

      double TimeLow = std::numeric_limits<double>::quiet_NaN();
      double TimeHigh = std::numeric_limits<double>::quiet_NaN();

      /** get timestamps */
      this->getTimeBelowOrEqual(ID, Timestamp, TimeLow);
      this->getTimeAboveOrEqual(ID, Timestamp, TimeHigh);

      /** catch nan */
      if (std::isnan(TimeLow))
      {
        TimestampClose = TimeHigh;
        return true;
      }
      if (std::isnan(TimeHigh))
      {
        TimestampClose = TimeLow;
        return true;
      }

      /** compare times */
      if (abs(TimeLow - Timestamp) < abs(TimeHigh - Timestamp))
      {
        TimestampClose = TimeLow;
      }
      else
      {
        TimestampClose = TimeHigh;
      }

      return true;
    }

    int countTimes(const KeyType &ID) const
    {
      if (!this->checkID(ID))
      {
        return 0;
      }

      return DataStreams.at(ID).size();
    }

    std::vector<KeyType> getKeysAll() const
    {
      std::vector<KeyType> Keys;
      for (auto it = DataStreams.begin(); it != DataStreams.end(); ++it)
      {
        Keys.push_back(it->first);
      }
      if (Keys.empty())
      {
        PRINT_WARNING("Returned empty vector!");
      }
      return Keys;
    }

    std::vector<KeyType> getKeysAtTime(const double Timestamp) const
    {
      std::vector<KeyType> KeysAll, KeysAtT;
      KeysAll = this->getKeysAll();
      for (const KeyType &Key : KeysAll)
      {
        if (this->checkElement(Key, Timestamp))
        {
          KeysAtT.push_back(Key);
        }
      }
      if (KeysAtT.empty())
      {
        PRINT_WARNING("Returned empty vector!");
      }
      return KeysAtT;
    }

    std::vector<ObjectType> getElementsOfID(const KeyType &ID) const
    {
      std::vector<ObjectType> Objects;

      if (DataStreams.count(ID) > 0)
      {
        for (auto it = DataStreams.at(ID).begin(); it != DataStreams.at(ID).end(); ++it)
        {
          Objects.push_back(it->second);
        }
      }

      if (Objects.empty())
      {
        PRINT_WARNING("Returned empty vector!");
      }
      return Objects;
    }

    std::vector<ObjectType> getElements(const KeyType &ID, const double Timestamp) const
    {
      std::vector<ObjectType> Objects;

      if (this->checkID(ID))
      {
        auto Range = DataStreams.at(ID).equal_range(Timestamp);
        for (auto it = Range.first; it != Range.second; ++it)
        {
          Objects.push_back(it->second);
        }
      }

      if (Objects.empty())
      {
        PRINT_WARNING("Returned empty vector!");
      }
      return Objects;
    }

    std::vector<ObjectType> getElementsBetween(const KeyType &ID, const double TimeBegin, const double TimeEnd) const
    {
      std::vector<ObjectType> Objects;

      /** catch special case, where timestamps are identical */
      if (TimeBegin == TimeEnd)
      {
        return this->getElements(ID, TimeBegin);
      }

      if (this->checkID(ID))
      {
        /** identify true boarders */
        double TimeFirst, TimeLast;
        if (this->getTimeBelowOrEqual(ID, TimeEnd, TimeLast) == false)
        {
          PRINT_WARNING("Did not find upper bound of ", ID, " at ", TimeBegin);
          return Objects;
        }
        if (this->getTimeAboveOrEqual(ID, TimeBegin, TimeFirst) == false)
        {
          PRINT_WARNING("Did not find lower bound of ", ID, " at ", TimeBegin);
          return Objects;
        }

        if (TimeFirst > TimeLast)
        {
          PRINT_WARNING("There is no object between ", TimeBegin, "s and ", TimeEnd, "s of type ", ID);
          return Objects;
        }

        /** iterate over timestamps */
        double Time = TimeFirst;
        do
        {
          std::vector<ObjectType> CurrentObjects = this->getElements(ID, Time);
          Objects.insert(Objects.end(), CurrentObjects.begin(), CurrentObjects.end());

          /** exit loop if last timestamp is reached */
          if (Time == TimeLast)
          {
            break;
          }
        } while (this->getTimeNext(ID, Time, Time));
      }
      else
      {
        PRINT_ERROR("There is no element with type: ", ID);
      }

      if (Objects.empty())
      {
        PRINT_WARNING("Returned empty vector!");
      }

      return Objects;
    }

    bool getUniqueIDs(const KeyType &ID, std::vector<UniqueID> &IDs) const
    {
      if (this->checkID(ID))
      {
        /** loop over timestamps */
        const ObjectStream &StreamRef = DataStreams.at(ID);
        for (auto it = StreamRef.begin(); it != StreamRef.end(); it = StreamRef.upper_bound(it->first))
        {
          /** loop over elements at one timestamp */
          for (int n = 0; n < static_cast<int>(this->countElement(ID, it->first)); ++n)
          {
            IDs.push_back(UniqueID(ID, it->first, n));
          }
        }
        return true;
      }

      PRINT_ERROR("There is no ID: ", ID);
      return false;
    }

    bool getTimesOfID(const KeyType &ID, std::vector<double> &Times) const
    {
      if (this->checkID(ID))
      {
        const ObjectStream &StreamRef = DataStreams.at(ID);
        for (auto it = StreamRef.begin(); it != StreamRef.end(); it = StreamRef.upper_bound(it->first))
        {
          Times.push_back(it->first);
        }
        return true;
      }

      PRINT_ERROR("There is no ID: ", ID);
      return false;
    }

    bool getTimesBetween(const KeyType &ID, const double StartTime, const double EndTime, std::vector<double> &Times) const
    {
      if (this->checkID(ID))
      {
        double Start, End;
        if (this->FindBordersEqual(ID, StartTime, EndTime, Start, End))
        {
          double Current = Start;
          while (Current <= End)
          {
            Times.push_back(Current);

            if (this->getTimeNext(ID, Current, Current) == false)
            {
              break; /**< exit if no further timestamp is available */
            }
          }
        }
        else
        {
          PRINT_WARNING("Could not find timestamps between ", StartTime, " and ", EndTime, " for ", ID);
          return false;
        }
      }
      else
      {
        PRINT_ERROR("There is no ID: ", ID);
        return false;
      }
      return true;
    }

    bool getTimesBelowOrEqual(const KeyType &ID, const double EndTime, std::vector<double> &Times) const
    {
      double Start;
      if (this->getTimeFirst(ID, Start))
      {
        double End;
        if (this->getTimeBelowOrEqual(ID, EndTime, End))
        {
          double Current = Start;
          bool HasNext = true;
          while (Current <= End && HasNext)
          {
            Times.push_back(Current);
            HasNext = this->getTimeNext(ID, Current, Current);
          }
        }
        else
        {
          PRINT_WARNING("Could not find timestamps before ", EndTime, " for ", ID);
          return false;
        }
      }
      else
      {
        PRINT_ERROR("There is no ID: ", ID);
        return false;
      }
      return true;
    }

    /** functions for range based for-loops */
    auto begin()
    {
      return DataStreams.begin();
    }

    const auto begin() const
    {
      return DataStreams.begin();
    }

    auto end()
    {
      return DataStreams.end();
    }

    const auto end() const
    {
      return DataStreams.end();
    }

    /** combine two lists*/
    void merge(DataSet<KeyType, ObjectType> &List)
    {
      for (auto &Map : List)
      {
        for (auto &Element : Map.second)
        {
          this->addElement(Map.first, Element.first, Element.second);
        }
      }
    }

   protected:
    bool FindBordersEqual(const KeyType &ID, const double Start, const double End, double &StartTrue, double &EndTrue) const
    {
      if (Start > End)
      {
        PRINT_ERROR("Start: ", Start, " is grater than End: ", End);
        return false;
      }

      if (!this->getTimeAboveOrEqual(ID, Start, StartTrue))
      {
        PRINT_ERROR("There is no object above: ", Start);
        return false;
      }

      if (!this->getTimeBelowOrEqual(ID, End, EndTrue))
      {
        PRINT_ERROR("There is no object above: ", End);
        return false;
      }
      return true;
    }

    bool FindBorders(const KeyType &ID, const double Start, const double End, double &StartTrue, double &EndTrue) const
    {
      if (Start > End)
      {
        PRINT_ERROR("Start: ", Start, " is grater than End: ", End);
        return false;
      }

      if (!this->getTimeAbove(ID, Start, StartTrue))
      {
        PRINT_ERROR("There is no object above: ", Start);
        return false;
      }

      if (!this->getTimeBelow(ID, End, EndTrue))
      {
        PRINT_ERROR("There is no object above: ", End);
        return false;
      }
      return true;
    }

    std::map<KeyType, ObjectStream> DataStreams;

    /** for empty references */
    ObjectType NullObject;
  };

}  // namespace libRSF

#endif  // Data_SET_H
