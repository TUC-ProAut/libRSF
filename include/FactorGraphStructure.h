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
 * @file FactorGraphStructure.h
 * @author Tim Pfeifer
 * @date 24.07.2019
 * @brief Represents the complete structure of an factor graph.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORGRAPHSTRUCTURE_H
#define FACTORGRAPHSTRUCTURE_H

#include "FactorIDSet.h"
#include "Hash.h"
#include "Messages.h"
#include "StateDataSet.h"
#include "SensorDataSet.h"
#include "error_models/ErrorModel.h"

namespace libRSF
{


  class FactorGraphStructure
  {
    public:
      FactorGraphStructure(ceres::Problem *GraphPointer, StateDataSet *Data);
      virtual ~FactorGraphStructure() = default;

      /** define a structure that stores all relevant information */
      struct StateInfo
      {
        std::string Name;
        double Timestamp;
        int Number;
        DataType Type;
      };

      struct FactorInfo
      {
        FactorType Type;
        double Timestamp;
        int Number;
        int ErrorInputSize;
        int ErrorOutputSize;
        ErrorModelBase* ErrorModel = nullptr;
      };

      /** collect all information that are required to remove the BaseState from the graph */
      void getMarginalizationInfo(const std::vector<double*> &BaseStates,
                                  std::vector<double*> &ConnectedStates,
                                  std::vector<ceres::ResidualBlockId> &ConnectedFactors,
                                  std::vector<int> &StateDims,
                                  std::vector<int> &StateDimsLocal,
                                  std::vector<StateID> &StateIDs,
                                  std::vector<DataType> &StateTypes) const;


      template <typename ErrorType>
      void addFactor(const FactorType Type,
                     const double Timestamp,
                     const ceres::ResidualBlockId CeresID,
                     ErrorType* const ErrorModel,
                     const std::vector<StateID> &States,
                     const std::vector<double*> &StatePointers,
                     const std::vector<DataType> &StateTypes)
      {
        /** add to time dependent representation */
        FactorList_.addElement(Type, Timestamp, CeresID);

        /** create ID and add to unordered maps */
        const FactorID Factor(Type, Timestamp, FactorList_.countElement(Type, Timestamp)-1);

        /** collect factor information */
        FactorInfo Info;
        Info.Type = Type;
        Info.Timestamp = Timestamp;
        Info.Number = Factor.Number;
        Info.ErrorInputSize = ErrorModel->InputDim;
        Info.ErrorOutputSize = ErrorModel->OutputDim;
        Info.ErrorModel = static_cast<ErrorModelBase*>(ErrorModel);
        Factors_.emplace(CeresID, Info);

        /** collect state information */
        for(int n = 0; n < static_cast<int>(States.size()); n++)
        {
          if(States_.count(StatePointers.at(n)) == 0) /**< check if already existing */
          {
            StateInfo State;
            State.Name = States.at(n).ID;
            State.Timestamp = States.at(n).Timestamp;
            State.Number = States.at(n).Number;
            State.Type = StateTypes.at(n);

            States_.emplace(StatePointers.at(n), State);
          }
        }
      }

      void removeFactor(const FactorID &Factor);
      void removeFactor(ceres::ResidualBlockId Factor);
      void removeState(const StateID &State);

      /** query single variables */
      void getResidualID(const FactorID &Factor, ceres::ResidualBlockId &Residual) const;
      void getErrorModel(const FactorID &Factor, ErrorModelBase* &ErrorModel) const;
      void getErrorInputSize(const FactorID &Factor, int &ResidualSize) const;
      void getErrorOutputSize(const FactorID &Factor, int &ResidualSize) const;

      /** query multiple variables */
      void getFactorIDs(FactorType Type, std::vector<FactorID> &Factors) const;
      void getErrorModels(FactorType Type, std::vector<ErrorModelBase*> &Models) const;
      void getResidualIDs(FactorType Type, std::vector<ceres::ResidualBlockId> &Blocks) const;

      /** query connected things */
      void getFactorsOfState(const StateID &State, std::vector<FactorID> &Factors) const;

      /** query timestamps */
      void getTimesBetween(FactorType Type, double StartTime, double EndTime, std::vector<double> &Times) const;
      void getTimesBelow(FactorType Type, double EndTime, std::vector<double> &Times) const;
      bool getTimeFirst(FactorType Type, double& FirstTime) const;
      bool getTimeLast(FactorType Type, double& LastTime) const;

      /** query keys */
      void getFactorTypes(std::vector<FactorType> &Factors) const;

      /** count stuff */
      [[nodiscard]] int countFactor(FactorType Type, double Timestamp) const;
      [[nodiscard]] int countFactorType(FactorType Type) const;

      /** check stuff */
      [[nodiscard]] bool checkFactor(FactorType Type, double Timestamp, int Number = 0) const;
      [[nodiscard]] bool checkFactor(FactorType Type) const;

    private:

      /** mapping ceres --> libRSF */
      std::map<double*, StateInfo> States_;
      std::map<ceres::ResidualBlockId, FactorInfo> Factors_;

      /** mapping libRSF --> ceres */
      FactorIDSet FactorList_;
      StateDataSet * const Data_;

      /** mapping states <--> factors */
      ceres::Problem * const Graph_;
  };
}

#endif // FACTORGRAPHSTRUCTURE_H
