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

#include "FactorGraph.h"


namespace libRSF
{

  std::ostream &operator<<(std::ostream &out, const FactorType Factor)
  {
    return out << "Factor";
  }

  void StateList::add(string Type, double Timestamp)
  {
    _List.emplace_back(std::make_pair(Type, Timestamp));
  }

  void StateList::clear()
  {
    _List.clear();
  }

  FactorGraph::FactorGraph()
  {}

  FactorGraph::FactorGraph(ceres::Problem::Options Options) : _Graph(Options)
  {}

  FactorGraph::~FactorGraph()
  {}

  void FactorGraph::solve()
  {
    ceres::Solve(_SolverOptions, &_Graph, &_Report);
  }

  void FactorGraph::solve(ceres::Solver::Options Options)
  {
    ceres::Solve(Options, &_Graph, &_Report);
  }

  void FactorGraph::addState(string Name, StateType Type, double Timestamp)
  {
    StateData Element(Type, Timestamp);
    addState(Name, Element);
  }

  void FactorGraph::addState(string Name, StateData &Element)
  {
    _StateData.addElement(Name, Element);
    double Timestamp = Element.getTimeStamp();

    /** add state vector as parameter block with local parametrization if required */
    switch(_StateData.getElement(Name, Timestamp, 0).getType())
    {
    case StateType::Angle:
      _Graph.AddParameterBlock(_StateData.getElement(Name, Timestamp, 0).getMeanPointer(),
                               _StateData.getElement(Name, Timestamp, 0).getMean().size(),
                               AngleLocalParameterization::Create());
      break;

    default:
      _Graph.AddParameterBlock(_StateData.getElement(Name, Timestamp, 0).getMeanPointer(),
                               _StateData.getElement(Name, Timestamp, 0).getMean().size());
      break;
    }
  }

  void FactorGraph::printReport()
  {
    std::cout << _Report.FullReport() << "\n";
  }

  void FactorGraph::initValues(FactorType CurrentFactorType, std::vector<double*> &StatePointers, MeasurementList &Measurements)
  {
    /** use predictor functions to initialize data */
    /** use predictor functions to initialize data */
    switch(CurrentFactorType)
    {

    case FactorType::ConstDrift1:
      PredictConstantDrift<double, 1>(StatePointers[0],
                                      StatePointers[1],
                                      StatePointers[2],
                                      StatePointers[3],
                                      Measurements.get(SensorType::DeltaTime).getMean()[0]);
      break;

    case FactorType::Odom4:
      PredictOdometry4DOFGlobal<double>(StatePointers[0],
                                        StatePointers[1],
                                        StatePointers[2],
                                        StatePointers[3],
                                        Measurements.get(SensorType::Odom3).getMean(),
                                        Measurements.get(SensorType::DeltaTime).getMean()[0]);
      break;

    case FactorType::Odom2Diff:
      PredictMotionDifferential2D<double>(StatePointers[0],
                                          StatePointers[1],
                                          StatePointers[2],
                                          StatePointers[3],
                                          Measurements.get(SensorType::Odom2Diff).getMean(),
                                          Measurements.get(SensorType::Odom2Diff).getValue(SensorElement::WheelBase)[0],
                                          Measurements.get(SensorType::DeltaTime).getMean()[0]);
      break;

    default:
      /** do nothing */
      break;
    }
  }

  StateDataSet &FactorGraph::getStateData()
  {
    return _StateData;
  }

  void FactorGraph::removeState(string Name, double Timestamp)
  {
    if (_StateData.checkElement(Name, Timestamp))
    {
      /** remove from ceres::problem */
      for (int StateNumber = _StateData.countElement(Name, Timestamp); StateNumber > 0; --StateNumber)
      {
        _Graph.RemoveParameterBlock(_StateData.getElement(Name, Timestamp, StateNumber-1).getMeanPointer());
      }

      /** remove from our StateDataSet */
      _StateData.removeElement(Name, Timestamp);
    }
    else
    {
      std::cerr << "In FactorGraph::removeState: State doesn't exist: " << Name << " " << Timestamp << " " << std::endl;
    }
  }

  void FactorGraph::removeStatesOutsideWindow(string Name, double TimeWindow, double CurrentTime)
  {
    double CutTime = CurrentTime - TimeWindow;
    double Timestamp;
    bool TimestampExists;

    /** loop over timestamps */
    TimestampExists = _StateData.getFirstTimestamp(Name, Timestamp);
    if(TimestampExists)
    {
      while((Timestamp <= CutTime) && TimestampExists)
      {
        double TimestampTemp = Timestamp;
        TimestampExists = _StateData.getNextTimestamp(Name, Timestamp, Timestamp);
        removeState(Name, TimestampTemp);
      }
    }
    else
    {
      std::cerr << "In FactorGraph::removeStatesOutsideWindow: State Type doesn't exist: " << Name << std::endl;
    }
  }

  void FactorGraph::removeAllStatesOutsideWindow(double TimeWindow, double CurrentTime)
  {
    for (auto const& State : _StateData)
    {
      removeStatesOutsideWindow(State.first, TimeWindow, CurrentTime);
    }
  }

  void FactorGraph::removeFactor(FactorType CurrentFactorType, double Timestamp)
  {
    if (_FactorList.checkElement(CurrentFactorType, Timestamp))
    {
      /** loop over factors */
      ceres::ResidualBlockId FactorID;
      for (size_t FactorNumber = _FactorList.countElement(CurrentFactorType, Timestamp); FactorNumber > 0; FactorNumber--)
      {
        FactorID = _FactorList.getElement(CurrentFactorType, Timestamp, FactorNumber-1);
        _Graph.RemoveResidualBlock(FactorID);
        _ErrorModelList.erase(FactorID);
      }
      _FactorList.removeElement(CurrentFactorType, Timestamp);
    }
    else
    {
      std::cerr << "In removeFactor: Factor doesn't exist: " << CurrentFactorType << " " << Timestamp << " " << std::endl;
    }
  }

  void FactorGraph::removeFactorsOutsideWindow(FactorType CurrentFactorType, double TimeWindow, double CurrentTime)
  {
    double CutTime = CurrentTime - TimeWindow;
    double Timestamp;
    bool TimestampExists;

    /** loop over timestamps */
    TimestampExists = _FactorList.getFirstTimestamp(CurrentFactorType, Timestamp);

    if(TimestampExists)
    {
      while((Timestamp <= CutTime) && TimestampExists)
      {
        double TimestampTemp = Timestamp;
        TimestampExists = _FactorList.getNextTimestamp(CurrentFactorType, Timestamp, Timestamp);
        removeFactor(CurrentFactorType, TimestampTemp);
      }
    }
    else
    {
      std::cerr << "In removeFactorsOutsideWindow: Factor Type doesn't exist: " << CurrentFactorType << std::endl;
    }
  }

  void FactorGraph::removeAllFactorsOutsideWindow(double TimeWindow, double CurrentTime)
  {
    for(auto const &Factor : _FactorList)
    {
      removeFactorsOutsideWindow(Factor.first, TimeWindow, CurrentTime);
    }
  }

  void FactorGraph::enableErrorModel(FactorType CurrentFactorType)
  {
    /** loop over all models */
    std::vector<ceres::ResidualBlockId> FactorIDList = _FactorList.getObjects(CurrentFactorType);
    for (ceres::ResidualBlockId FactorID : FactorIDList)
    {
      _ErrorModelList.at(FactorID)->enable();
    }
  }

  void FactorGraph::disableErrorModel(FactorType CurrentFactorType)
  {
    /** loop over all models */
    std::vector<ceres::ResidualBlockId> FactorIDList = _FactorList.getObjects(CurrentFactorType);
    for (ceres::ResidualBlockId FactorID : FactorIDList)
    {
      _ErrorModelList.at(FactorID)->disable();
    }
  }

  void FactorGraph::computeUnweightedError(FactorType CurrentFactorType, std::vector<double> &ErrorData)
  {
    disableErrorModel(CurrentFactorType);

    /** configure evaluation */
    ceres::Problem::EvaluateOptions Options;
    Options.num_threads = 8;
    Options.residual_blocks = _FactorList.getObjects(CurrentFactorType);
    Options.apply_loss_function = false;

    /** compute the errors */
    _Graph.Evaluate(Options, nullptr, &ErrorData, nullptr, nullptr);

    enableErrorModel(CurrentFactorType);
  }
}
