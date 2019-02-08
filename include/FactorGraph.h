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
 * @file FactorGraph.h
 * @author Tim Pfeifer
 * @date 06.06.2018
 * @brief Main class that contains the optimization problem and several interfaces and method to interact with it.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORGRAPH_H
#define FACTORGRAPH_H

#include <ceres/ceres.h>

#include "SensorData.h"
#include "StateDataSet.h"
#include "SensorDataSet.h"
#include "AngleLocalParametrization.h"
#include "FileAccess.h"
#include "ListInTime.h"

#include "error_models/ErrorModel.h"
#include "error_models/Gaussian.h"
#include "error_models/MaxMixture.h"
#include "error_models/SumMixture.h"
#include "error_models/LossFunction.h"

#include "factors/MeasurementFactor.h"
#include "factors/ConstantDriftFactor.h"
#include "factors/RangeFactor.h"
#include "factors/PseudorangeFactor.h"
#include "factors/OdometryFactor2DDifferential.h"
#include "factors/OdometryFactor3D.h"

#include "predictors/PredictConstantDrift.h"
#include "predictors/PredictOdometry.h"
#include "predictors/PredictOdometry2DDifferential.h"


namespace libRSF
{
  enum class FactorType {
    ConstVal1, ConstVal2, ConstVal3,
    ConstDrift1, ConstDrift2, ConstDrift3,
    BetweenValue1, BetweenValue2, BetweenValue3,
    BetweenPose2,
    Range2, Range3, Pseudorange2, Pseudorange3,
    Odom2, Odom6, Odom4, Odom2Diff, Odom4Local,
    Prior1, Prior2, Prior3, Prior4,
    IMUPretintegration
  };

  std::ostream &operator<<(std::ostream &out, const FactorType Factor);

  struct StateList
  {
    void add(string Type, double Timestamp);
    void clear();
    std::vector<std::pair<string, double>> _List;
  };

  typedef ceres::ResidualBlockId FactorIDType;

  class FactorGraph
  {
    public:
      /** Default constructor */
      FactorGraph();
      explicit FactorGraph(ceres::Problem::Options Options);
      /** Default destructor */
      virtual ~FactorGraph();

      typedef ListInTime<FactorType, FactorIDType> FactorListType;
      typedef std::map<FactorIDType, SensorData> SensorDataListType;
      typedef std::map<FactorIDType, ErrorModelBase*> ErrorModelListType;

      /** access to single states */
      void addState(string Name, StateType Type, double Timestamp);
      void addState(string Name, StateData &Element);

      /** access to complete state data */
      StateDataSet &getStateData();

      /** access to error model data */
      void enableErrorModel(FactorType CurrentFactorType);
      void disableErrorModel(FactorType CurrentFactorType);

      /** replace error model off all factors given by ID */
      template <typename ErrorType>
      void setNewErrorModel(std::vector<FactorIDType> FactorIDList, ErrorType NoiseModel)
      {
        for (FactorIDType FactorID : FactorIDList)
        {
          *static_cast<ErrorType*>(_ErrorModelList.at(FactorID)) = NoiseModel;
        }
      }
      /** replace error model off all factors of a specific type */
      template <typename ErrorType>
      void setNewErrorModel(FactorType CurrentFactorType, ErrorType NoiseModel)
      {
        std::vector<FactorIDType> FactorIDList = _FactorList.getObjects(CurrentFactorType);
        setNewErrorModel(FactorIDList, NoiseModel);
      }

      /** add and remove factors */
      template <typename ErrorType>
      void addFactor(FactorType CurrentFactorType, StateList &States, ErrorType &NoiseModel, ceres::LossFunction* RobustLoss = nullptr)
      {
        MeasurementList EmptyList;
        addFactor(CurrentFactorType, States, NoiseModel, EmptyList, RobustLoss);
      }

      template <typename ErrorType>
      void addFactor(FactorType CurrentFactorType, StateList &States, SensorData &Measurement1, ErrorType &NoiseModel, ceres::LossFunction* RobustLoss = nullptr)
      {
        MeasurementList NewList;
        NewList.add(Measurement1);
        addFactor(CurrentFactorType, States, NoiseModel, NewList, RobustLoss);
      }

      template <typename ErrorType>
      void addFactor(FactorType CurrentFactorType, StateList &States, SensorData &Measurement1, SensorData &Measurement2, ErrorType &NoiseModel, ceres::LossFunction* RobustLoss = nullptr)
      {
        MeasurementList NewList;
        NewList.add(Measurement1);
        NewList.add(Measurement2);
        addFactor(CurrentFactorType, States, NoiseModel, NewList, RobustLoss);
      }

      /** for sliding window */
      void removeFactor(FactorType CurrentFactorType, double Timestamp);
      void removeFactorsOutsideWindow(FactorType CurrentFactorType, double TimeWindow, double CurrentTime);
      void removeAllFactorsOutsideWindow(double TimeWindow, double CurrentTime);

      /** solve problem */
      void solve();
      void solve(ceres::Solver::Options Options);

      /** remove old states*/
      void removeState(string Name, double Timestamp);
      void removeStatesOutsideWindow(string Name, double TimeWindow, double CurrentTime);
      void removeAllStatesOutsideWindow(double TimeWindow, double CurrentTime);

      /** compute covariances */
      bool computeCovariance(string Name, double Timestamp);
      bool computeCovariance(string Name);

      /** evaluate errors */
      void computeUnweightedError(FactorType CurrentFactorType, std::vector<double> &ErrorData);

      /** access solver options */
      void setSolverOptions(ceres::Solver::Options Options);
      ceres::Solver::Options getSolverOptions();

      /** access solver summary */
      void printReport();

    protected:


    private:
      /** access predictors to initialize values */
      void initValues(FactorType CurrentFactorType, std::vector<double*> &StatePointers, MeasurementList &Measurements);

      /** helper function to create variadic templated cost functions */
      template<int NoiseModelOutputDim, typename CurrentFactorType,  int... FactorStateDims, int... ErrorModelStateDims>
      auto makeAutoDiffCostFunction(CurrentFactorType *Factor, std::integer_sequence<int, FactorStateDims...>, std::integer_sequence<int, ErrorModelStateDims...>)
      {
        return new ceres::AutoDiffCostFunction<CurrentFactorType, NoiseModelOutputDim, FactorStateDims... , ErrorModelStateDims...> (Factor);
      }

      /** add and remove factors */
      template <typename ErrorType, typename CurrentFactorType>
      void addFactorGeneric (ErrorType &NoiseModel, std::vector<double*> &StatePointers, MeasurementList &Measurements, FactorType FactorTypeEnum, ceres::LossFunction* RobustLoss, double Timestamp)
      {
        /** create libRSF factor object */
        auto Factor = new CurrentFactorType(NoiseModel, Measurements);

        /** wrap it in ceres cost function */
        auto CostFunction = makeAutoDiffCostFunction<NoiseModel._OutputDim, CurrentFactorType> (Factor,
                                                                                                typename CurrentFactorType::_StateDims{},
                                                                                                typename ErrorType::_StateDims{});

        /** add it to the estimation problem  */
        auto FactorID = _Graph.AddResidualBlock(CostFunction,
                                                RobustLoss,
                                                StatePointers);

        _FactorList.addElement(FactorTypeEnum, Timestamp, FactorID);
        _ErrorModelList.emplace(FactorID, Factor->getErrorModel());
      }

      template <typename ErrorType>
      void addFactor(FactorType CurrentFactorType, StateList &States, ErrorType &NoiseModel, MeasurementList &Measurements, ceres::LossFunction* RobustLoss)
      {

        /** build list of states */
        std::vector<double*> StatePointers;

        for(const std::pair<string, double> &State : States._List)
        {
          size_t StateNumber = _StateData.countElement(State.first, State.second) - 1;
          StatePointers.emplace_back(_StateData.getElement(State.first, State.second, StateNumber).getMeanPointer());
        }

        double Timestamp = _StateData.getElement(States._List[0].first, States._List[0].second, 0).getTimeStamp();

        /** set initial values if the factor connects states over time */
        initValues(CurrentFactorType, StatePointers, Measurements);

        /** TODO: simplify!!!*/
        switch(CurrentFactorType)
        {
        case FactorType::ConstDrift1:
          addFactorGeneric<ErrorType, libRSF::ConstantDriftFactorBase<ErrorType,1>>(NoiseModel, StatePointers, Measurements, CurrentFactorType, RobustLoss, Timestamp);
          break;

        case FactorType::Range2:
          addFactorGeneric<ErrorType, libRSF::RangeFactorBase<ErrorType,2>>(NoiseModel, StatePointers, Measurements, CurrentFactorType, RobustLoss, Timestamp);
          break;

        case FactorType::Pseudorange3:
          addFactorGeneric<ErrorType, libRSF::PseudorangeFactorBase<ErrorType,3>>(NoiseModel, StatePointers, Measurements, CurrentFactorType, RobustLoss, Timestamp);
          break;

        case FactorType::Odom2Diff:
          addFactorGeneric<ErrorType, libRSF::OdometryFactor2DDifferential<ErrorType>>(NoiseModel, StatePointers, Measurements, CurrentFactorType, RobustLoss, Timestamp);
          break;

        case FactorType::Odom4:
          addFactorGeneric<ErrorType, libRSF::OdometryFactor3D4DOF<ErrorType>>(NoiseModel, StatePointers, Measurements, CurrentFactorType, RobustLoss, Timestamp);
          break;

        default:
          std::cerr << "Error in FactorGraph.addFactor(): FactorType::??? has no handler!" << std::endl;
          break;
        }
      };

      ceres::Problem _Graph;
      ceres::Solver::Summary _Report;
      ceres::Solver::Options _SolverOptions;

      StateDataSet _StateData;                      /**< holds all state variables */
      FactorListType _FactorList;                   /**< holds the IDs (a ceres data type) of all factors */
      ErrorModelListType _ErrorModelList;           /**< holds pointers to all error models */
      SensorDataListType _SensorDataList;               /**< holds pointers to all measurement lists of all factors */
  };
}

#endif // FACTORGRAPH_H
