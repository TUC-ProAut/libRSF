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

#include "FactorGraphStructure.h"

namespace libRSF
{
  FactorGraphStructure::FactorGraphStructure(ceres::Problem *GraphPointer, StateDataSet * Data): Data_(Data), Graph_(GraphPointer)
  {}

  void FactorGraphStructure::getMarginalizationInfo(const std::vector<double*> &BaseStates,
                                              std::vector<double*> &ConnectedStates,
                                              std::vector<ceres::ResidualBlockId> &ConnectedFactors,
                                              std::vector<int> &StateDims,
                                              std::vector<int> &StateDimsLocal,
                                              std::vector<StateID> &StateIDs,
                                              std::vector<DataType> &StateTypes) const
  {
    /** sets to store unique values */
    std::set<double*> BaseStatePointers;
    std::set<ceres::ResidualBlockId> ConnectedFactorsIDs;

    /** get all connected factors */
    for (double* const State : BaseStates)
    {
      /** safety check */
      if(!Graph_->HasParameterBlock(State))
      {
        PRINT_ERROR("State is not part of graph!");
        return;
      }

      /** store pointer for check */
      if(BaseStatePointers.count(State) != 0)
      {
        PRINT_ERROR("Duplicated pointer for marginalized states!");
        return;
      }
      BaseStatePointers.emplace(State);

      /** query residual IDs */
      std::vector<ceres::ResidualBlockId> Factors;
      Graph_->GetResidualBlocksForParameterBlock(State, &Factors);
      for(ceres::ResidualBlockId Factor : Factors)
      {
        /** add only new factors */
        if(ConnectedFactorsIDs.count(Factor) == 0)
        {
          ConnectedFactorsIDs.emplace(Factor);
        }
      }
    }

    /** get all connected states */
    std::set<double*> ConnectedStatePointers;
    for (const ceres::ResidualBlockId Factor : ConnectedFactorsIDs)
    {
      std::vector<double*> CurrentStatePointers;

      /** query states that are connected with a certain factor */
      Graph_->GetParameterBlocksForResidualBlock(Factor, &CurrentStatePointers);

      /** add only unique states */
      for (double* StatePointer : CurrentStatePointers)
      {
        if(ConnectedStatePointers.count(StatePointer) == 0 && BaseStatePointers.count(StatePointer) == 0)
        {
          ConnectedStatePointers.emplace(StatePointer);
        }
      }

      /** copy from set to vector */
      ConnectedFactors.emplace_back(Factor);
    }

    /** copy into vector */
    for (double* const State : ConnectedStatePointers)
    {
      ConnectedStates.emplace_back(State);
      StateDims.emplace_back(Graph_->ParameterBlockSize(State));
      StateDimsLocal.emplace_back(Graph_->ParameterBlockLocalSize(State));

      /** find state info */
      StateInfo Info = States_.at(State);
      StateIDs.emplace_back(StateID(Info.Name, Info.Timestamp, Info.Number));
      StateTypes.emplace_back(Info.Type);
    }
  }

  void FactorGraphStructure::getFactorsOfState(const StateID &State, std::vector<FactorID> &Factors) const
  {
    if (!Data_->checkElement(State.ID, State.Timestamp, State.Number))
    {
      PRINT_ERROR("State does not exist: ", State.ID, " ", State.Timestamp, " ", State.Number);
      return;
    }

    /** get state pointer */
    double* StatePointer;
    StatePointer = Data_->getElement(State.ID, State.Timestamp, State.Number).getMeanPointer();

    /** get connected residual IDs */
    std::vector<ceres::ResidualBlockId> Residuals;
    Graph_->GetResidualBlocksForParameterBlock(StatePointer, &Residuals);

    /** translate to factor IDs */
    Factors.clear();
    for(const ceres::ResidualBlockId Res: Residuals)
    {
      FactorID Factor(Factors_.at(Res).Type, Factors_.at(Res).Timestamp, Factors_.at(Res).Number);
      Factors.push_back(Factor);
    }

  }

  void FactorGraphStructure::removeState(const StateID &State)
  {
    /** get raw pointer*/
    double* StatePointer = Data_->getElement(State.ID, State.Timestamp, State.Number).getMeanPointer();

    /** remove state info */
    States_.erase(StatePointer);

    /** get connected factors */
    std::vector<ceres::ResidualBlockId> Factors;
    Graph_->GetResidualBlocksForParameterBlock(StatePointer, &Factors);

    /** remove connected factors */
    for (auto *Factor: Factors)
    {
      this->removeFactor(Factor);
    }
  }

  void FactorGraphStructure::removeFactor(const ceres::ResidualBlockId Factor)
  {
    /** clear mapping libRSF --> ceres */
    FactorInfo Info = Factors_.at(Factor);
    FactorList_.removeElement(Info.Type, Info.Timestamp, Info.Number);

    /** clear mapping ceres --> libRSF */
    Factors_.erase(Factor);

    /** decrement index in factor info to correct the number of the elements above the deleted one*/
    for (int n = Info.Number; n < FactorList_.countElement(Info.Type, Info.Timestamp); n++)
    {
      Factors_.at(FactorList_.getElement(Info.Type, Info.Timestamp, n)).Number--;
    }
  }

  void FactorGraphStructure::removeFactor(const FactorID &Factor)
  {
    this->removeFactor(FactorList_.getElement(Factor.ID, Factor.Timestamp, Factor.Number));
  }

  void FactorGraphStructure::getResidualID(const FactorID &Factor, ceres::ResidualBlockId &Residual) const
  {
    FactorList_.getElement(Factor.ID, Factor.Timestamp, Factor.Number, Residual);
  }

  void FactorGraphStructure::getErrorModel(const FactorID &Factor, ErrorModelBase* &ErrorModel) const
  {
    ceres::ResidualBlockId ID;
    this->getResidualID(Factor, ID);
    ErrorModel = Factors_.at(ID).ErrorModel;
  }

  void FactorGraphStructure::getErrorInputSize(const FactorID &Factor, int &ResidualSize) const
  {
    ceres::ResidualBlockId ID;
    this->getResidualID(Factor, ID);
    ResidualSize = Factors_.at(ID).ErrorInputSize;
  }

  void FactorGraphStructure::getErrorOutputSize(const FactorID &Factor, int &ResidualSize) const
  {
    ceres::ResidualBlockId ID;
    this->getResidualID(Factor, ID);
    ResidualSize = Factors_.at(ID).ErrorOutputSize;
  }

  void FactorGraphStructure::getFactorIDs(const FactorType Type, std::vector<FactorID> &Factors) const
  {
    FactorList_.getUniqueIDs(Type, Factors);
  }

  void FactorGraphStructure::getErrorModels(const FactorType Type, std::vector<ErrorModelBase*> &Models) const
  {
    /**get IDs */
    std::vector<FactorID> Factors;
    this->getFactorIDs(Type, Factors);

    /** translate to models */
    for(auto const &Factor: Factors)
    {
      ErrorModelBase* Model;
      this->getErrorModel(Factor, Model);
      Models.push_back(Model);
    }
  }

  void FactorGraphStructure::getResidualIDs(const FactorType Type, std::vector<ceres::ResidualBlockId> &Blocks) const
  {
    Blocks = FactorList_.getElementsOfID(Type);
  }

  void FactorGraphStructure::getTimesBetween(const FactorType Type, const double StartTime, const double EndTime, std::vector<double> &Times) const
  {
    FactorList_.getTimesBetween(Type, StartTime, EndTime, Times);
  }

  void FactorGraphStructure::getTimesBelow(const FactorType Type, const double EndTime, std::vector<double> &Times) const
  {
    double StartTime;
    if(FactorList_.getTimeFirst(Type, StartTime))
    {
      if(StartTime <= EndTime)
      {
        this->getTimesBetween(Type, StartTime, EndTime, Times);
      }
      else
      {
        PRINT_WARNING("There is now element of type ", Type, " below timestamp ", EndTime, ".");
      }
    }
  }

  bool FactorGraphStructure::getTimeFirst(const FactorType Type, double& FirstTime) const
  {
    return FactorList_.getTimeFirst(Type, FirstTime);
  }

  bool FactorGraphStructure::getTimeLast(const FactorType Type, double& LastTime) const
  {
    return FactorList_.getTimeLast(Type, LastTime);
  }

  void FactorGraphStructure::getFactorTypes(std::vector<FactorType> &Factors) const
  {
    Factors = FactorList_.getKeysAll();
  }

  int FactorGraphStructure::countFactor(const FactorType Type, const double Timestamp) const
  {
    return FactorList_.countElement(Type, Timestamp);
  }

  int FactorGraphStructure::countFactorType(const FactorType Type) const
  {
    return FactorList_.countElements(Type);
  }

  bool FactorGraphStructure::checkFactor(const FactorType Type, const double Timestamp, const int Number) const
  {
    return FactorList_.checkElement(Type, Timestamp, Number);
  }

  bool FactorGraphStructure::checkFactor(const FactorType Type) const
  {
    return FactorList_.checkID(Type);
  }
}
