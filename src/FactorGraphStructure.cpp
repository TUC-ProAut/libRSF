/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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
  FactorGraphStructure::FactorGraphStructure(ceres::Problem *GraphPointer, StateDataSet * Data): _Data(Data), _Graph(GraphPointer)
  {}

  void FactorGraphStructure::getMarginalizationInfo(const std::vector<double*> &BaseStates,
                                              std::vector<double*> &ConnectedStates,
                                              std::vector<ceres::ResidualBlockId> &ConnectedFactors,
                                              std::vector<int> &StateDims,
                                              std::vector<int> &StateDimsLocal,
                                              std::vector<StateID> &StateIDs,
                                              std::vector<StateType> &StateTypes) const
  {
    /** sets to store unique values */
    std::set<double*> BaseStatePointers;
    std::set<ceres::ResidualBlockId> ConnectedFactorsIDs;

    /** get all connected factors */
    for (double* const State : BaseStates)
    {
      /** safety check */
      if(_Graph->HasParameterBlock(State) == false)
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
      _Graph->GetResidualBlocksForParameterBlock(State, &Factors);
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
      _Graph->GetParameterBlocksForResidualBlock(Factor, &CurrentStatePointers);

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
      StateDims.emplace_back(_Graph->ParameterBlockSize(State));
      StateDimsLocal.emplace_back(_Graph->ParameterBlockLocalSize(State));

      /** find state info */
      StateInfo Info = _States.at(State);
      StateIDs.emplace_back(StateID(Info.Name, Info.Timestamp, Info.Number));
      StateTypes.emplace_back(Info.Type);
    }
  }

  void FactorGraphStructure::getFactorsOfState(const StateID &State, std::vector<FactorID> &Factors) const
  {
    if (_Data->checkElement(State.ID, State.Timestamp, State.Number) == false)
    {
      PRINT_ERROR("State does not exist: ", State.ID, " ", State.Timestamp, " ", State.Number);
      return;
    }

    /** get state pointer */
    double* StatePointer;
    StatePointer = _Data->getElement(State.ID, State.Timestamp, State.Number).getMeanPointer();

    /** get connected residual IDs */
    std::vector<ceres::ResidualBlockId> Residuals;
    _Graph->GetResidualBlocksForParameterBlock(StatePointer, &Residuals);

    /** translate to factor IDs */
    Factors.clear();
    for(const ceres::ResidualBlockId Res: Residuals)
    {
      FactorID Factor(_Factors.at(Res).Type, _Factors.at(Res).Timestamp, _Factors.at(Res).Number);
      Factors.push_back(Factor);
    }

  }

  void FactorGraphStructure::removeState(const StateID &State)
  {
    /** get raw pointer*/
    double* StatePointer = _Data->getElement(State.ID, State.Timestamp, State.Number).getMeanPointer();

    /** remove state info */
    _States.erase(StatePointer);

    /** get connected factors */
    std::vector<ceres::ResidualBlockId> Factors;
    _Graph->GetResidualBlocksForParameterBlock(StatePointer, &Factors);

    /** remove connected factors */
    for (auto Factor: Factors)
    {
      this->removeFactor(Factor);
    }
  }

  void FactorGraphStructure::removeFactor(const ceres::ResidualBlockId Factor)
  {
    /** clear mapping libRSF --> ceres */
    FactorInfo Info = _Factors.at(Factor);
    _FactorList.removeElement(Info.Type, Info.Timestamp, Info.Number);

    /** clear mapping ceres --> libRSF */
    _Factors.erase(Factor);

    /** decrement index in factor info to correct the number of the elements above the deleted one*/
    for (int n = Info.Number; n < _FactorList.countElement(Info.Type, Info.Timestamp); n++)
    {
      _Factors.at(_FactorList.getElement(Info.Type, Info.Timestamp, n)).Number--;
    }
  }

  void FactorGraphStructure::removeFactor(const FactorID &Factor)
  {
    this->removeFactor(_FactorList.getElement(Factor.ID, Factor.Timestamp, Factor.Number));
  }

  void FactorGraphStructure::getResidualID(const FactorID &Factor, ceres::ResidualBlockId &Residual) const
  {
    _FactorList.getElement(Factor.ID, Factor.Timestamp, Factor.Number, Residual);
  }

  void FactorGraphStructure::getErrorModel(const FactorID &Factor, ErrorModelBase* &ErroModel) const
  {
    ceres::ResidualBlockId ID;
    this->getResidualID(Factor, ID);
    ErroModel = _Factors.at(ID).ErrorModel;
  }

  void FactorGraphStructure::getErrorInputSize(const FactorID &Factor, int &ResidualSize) const
  {
    ceres::ResidualBlockId ID;
    this->getResidualID(Factor, ID);
    ResidualSize = _Factors.at(ID).ErrorInputSize;
  }

  void FactorGraphStructure::getErrorOutputSize(const FactorID &Factor, int &ResidualSize) const
  {
    ceres::ResidualBlockId ID;
    this->getResidualID(Factor, ID);
    ResidualSize = _Factors.at(ID).ErrorOutputSize;
  }

  void FactorGraphStructure::getFactorIDs(const FactorType Type, std::vector<FactorID> &Factors) const
  {
    _FactorList.getUniqueIDs(Type, Factors);
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
    Blocks = _FactorList.getElementsOfID(Type);
  }

  void FactorGraphStructure::getTimesBetween(const FactorType Type, const double StartTime, const double EndTime, std::vector<double> &Times) const
  {
    _FactorList.getTimesBetween(Type, StartTime, EndTime, Times);
  }

  void FactorGraphStructure::getTimesBelow(const FactorType Type, const double EndTime, std::vector<double> &Times) const
  {
    double StartTime;
    if(_FactorList.getTimeFirst(Type, StartTime))
    {
      this->getTimesBetween(Type, StartTime, EndTime, Times);
    }
  }

  bool FactorGraphStructure::getTimeFirst(const FactorType Type, double& FirstTime) const
  {
    return _FactorList.getTimeFirst(Type, FirstTime);
  }

  bool FactorGraphStructure::getTimeLast(const FactorType Type, double& LastTime) const
  {
    return _FactorList.getTimeLast(Type, LastTime);
  }

  void FactorGraphStructure::getFactorTypes(std::vector<FactorType> &Factors) const
  {
    Factors = _FactorList.getKeysAll();
  }

  int FactorGraphStructure::countFactor(const FactorType Type, const double Timestamp) const
  {
    return _FactorList.countElement(Type, Timestamp);
  }

  int FactorGraphStructure::countFactorType(const FactorType Type) const
  {
    return _FactorList.countElements(Type);
  }

  bool FactorGraphStructure::checkFactor(const FactorType Type, const double Timestamp, const double Number) const
  {
    return _FactorList.checkElement(Type, Timestamp, Number);
  }

  bool FactorGraphStructure::checkFactor(const FactorType Type) const
  {
    return _FactorList.checkID(Type);
  }
}
