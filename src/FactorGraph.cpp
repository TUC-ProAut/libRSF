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

#include "FactorGraph.h"


namespace libRSF
{
  void StateList::add(const string& Type, double Timestamp, int Number)
  {
    this->add(StateID(Type, Timestamp, Number));
  }

  void StateList::add(const StateID& State)
  { List_.emplace_back(State);
  }

  void StateList::clear()
  { List_.clear();
  }

  FactorGraph::FactorGraph() : Graph_(this->DefaultProblemOptions_),
        Structure_(&Graph_, &StateData_),
        SolverDuration_(0.0),
        CovarianceDuration_(0.0),
        SolverIterations_(0),
        MarginalizationDuration_(0.0)
  {}

  void FactorGraph::solve()
  {
    /** check if config is valid */
    std::string OptionsError;
    if (!SolverOptions_.IsValid(&OptionsError))
    {
      PRINT_ERROR("The given solver options are wrong: ", OptionsError);
    }
    else
    {
      /** call ceres to solve the optimization problem */
      ceres::Solve(SolverOptions_, &Graph_, &Report_);
      SolverDuration_ += Report_.total_time_in_seconds;
      SolverIterations_ +=
          Report_.num_successful_steps + Report_.num_unsuccessful_steps;
    }
  }

  void FactorGraph::solve(ceres::Solver::Options Options)
  {
    SolverOptions_ = Options;
    this->solve();
  }

  void FactorGraph::addState(const string& Name, DataType Type, double Timestamp)
  {
    Data Element(Type, Timestamp);
    addState(Name, Element);
  }

  void FactorGraph::addState(const string& Name, Data &Element)
  {
    StateData_.addElement(Name, Element);
    double Timestamp = Element.getTimestamp();
    int StateNumber = StateData_.countElement(Name, Timestamp) - 1;

    double* StatePointer =
        StateData_.getElement(Name, Timestamp, StateNumber).getMeanPointer();
    const int StateSize = static_cast<int>(
        StateData_.getElement(Name, Timestamp, StateNumber).getMean().size());

    /** add state vector as parameter block with local parametrization if required */
    switch (StateData_.getElement(Name, Timestamp, StateNumber).getType())
    {
      case DataType::Angle:
        Graph_.AddParameterBlock(StatePointer, StateSize, AngleLocalParameterization::Create());
        break;

      case DataType::UnitCircle:
        {
          /** initialize with 0 degree rotation */
          Vector2 Circle;
          Circle << 1, 0;
          StateData_.getElement(Name, Timestamp, StateNumber).setMean(Circle);

          Graph_.AddParameterBlock(StatePointer, StateSize, UnitCircleLocalParameterization::Create());
          break;
        }

      case DataType::Quaternion:
        {
          /** initialize with valid quaternion */
          Vector4 Quat;
          Quat << 0, 0, 0, 1; /**< x,y,z,w */
          StateData_.getElement(Name, Timestamp, StateNumber).setMean(Quat);

          Graph_.AddParameterBlock(StatePointer, StateSize, QuaternionLocalParameterization::Create());
          break;
        }

      case DataType::Pose2:
        {
          ceres::LocalParameterization *LocalParamPose2 = new ceres::ProductParameterization(new ceres::IdentityParameterization(2), AngleLocalParameterization::Create());
          Graph_.AddParameterBlock(StatePointer, StateSize, LocalParamPose2);
          break;
        }

      case DataType::Pose3:
        {
          /** initialize with valid quaternion */
          Vector7 Pose3;
          Pose3 << 0,0,0, 0,0,0,1;
          StateData_.getElement(Name, Timestamp, StateNumber).setMean(Pose3);

          ceres::LocalParameterization *LocalParamPose3 = new ceres::ProductParameterization(new ceres::IdentityParameterization(3), QuaternionLocalParameterization::Create());
          Graph_.AddParameterBlock(StatePointer, StateSize, LocalParamPose3);
          break;
        }

      case DataType::Switch:
        {
          /** initialize with one */
          StateData_.getElement(Name, Timestamp, StateNumber).setMean(Vector1::Identity());

          Graph_.AddParameterBlock(StatePointer, StateSize);

          /** limit between zero and one */
          Graph_.SetParameterLowerBound(
              StateData_.getElement(Name, Timestamp, StateNumber).getMeanPointer(), 0, 0.0);
          Graph_.SetParameterUpperBound(
              StateData_.getElement(Name, Timestamp, StateNumber).getMeanPointer(), 0, 1.0);
          break;
        }

      default:
        Graph_.AddParameterBlock(StatePointer, StateSize);
        break;
    }
  }

  void FactorGraph::addStateWithCheck(const string& Name, DataType Type, double Timestamp)
  {
    if (!this->getStateData().checkElement(Name, Timestamp))
    {
      this->addState(Name, Type, Timestamp);
    }
  }

  void FactorGraph::addIMUPreintegrationFactor(StateList List, const PreintegratedIMUResult& IMUState)
  {
    /** create noise model */
    GaussianFull<15> IMUNoiseModel;
    IMUNoiseModel.setCovarianceMatrix(IMUState.PreIntCov);

    /** add to factor graph */
    using FactorClassType = typename FactorTypeTranslator<FactorType::IMUPretintegration, GaussianFull<15>>::Type;
    addFactorGeneric_<GaussianFull<15>, FactorClassType> (IMUNoiseModel,
                                                          List.List_,
                                                          FactorType::IMUPretintegration,
                                                          nullptr,
                                                          true,
                                                          List.List_.front().Timestamp,
                                                          IMUState);
  }

  void FactorGraph::printReport() const
  {
    std::cout << Report_.FullReport() << "\n";
  }

  ceres::Solver::Summary FactorGraph::getSolverSummary() const
  {
    return Report_;
  }

  void FactorGraph::setConstant(const string& Name, const double Timestamp)
  {
    for (int StateNumber = StateData_.countElement(Name, Timestamp); StateNumber > 0; --StateNumber)
    {
      Graph_.SetParameterBlockConstant(
          StateData_.getElement(Name, Timestamp, StateNumber - 1).getMeanPointer());
    }
  }

  void FactorGraph::setVariable(const string& Name, const double Timestamp)
  {
    for (int StateNumber = StateData_.countElement(Name, Timestamp); StateNumber > 0; --StateNumber)
    {
      Graph_.SetParameterBlockVariable(
          StateData_.getElement(Name, Timestamp, StateNumber - 1).getMeanPointer());
    }
  }

  void FactorGraph::setSubsetConstant(const string& Name, const double Timestamp, const int Number, const std::vector<int> &ConstantIndex)
  {
    Graph_.SetParameterization(
        StateData_.getElement(Name, Timestamp, Number).getMeanPointer(),
                               new ceres::SubsetParameterization(
            StateData_.getElement(Name, Timestamp, Number).getMean().size(), ConstantIndex));
  }

  void FactorGraph::setUpperBound(const string &Name, const double Timestamp, const int StateNumber, const Vector &Bound)
  {
    const int Dim = static_cast<int>(
        StateData_.getElement(Name, Timestamp, StateNumber).getMean().size());

    if (Bound.size() == Dim)
    {
      for (int n = 0; n < Dim; n++)
      {
        Graph_.SetParameterUpperBound(
            StateData_.getElement(Name, Timestamp, StateNumber).getMeanPointer(), n, Bound(n));
      }
    }
    else
    {
      PRINT_ERROR("Dimension of bounding vector is ", Bound.size(), " instead of ", Dim, "!") ;
    }
  }

  void FactorGraph::setLowerBound(const string &Name, const double Timestamp, const int StateNumber, const Vector &Bound)
  {
    const int Dim = static_cast<int>(
        StateData_.getElement(Name, Timestamp, StateNumber).getMean().size());

    if (Bound.size() == Dim)
    {
      for (int n = 0; n < Dim; n++)
      {
        Graph_.SetParameterLowerBound(
            StateData_.getElement(Name, Timestamp, StateNumber).getMeanPointer(), n, Bound(n));
      }
    }
    else
    {
      PRINT_ERROR("Dimension of bounding vector is ", Bound.size(), " instead of ", Dim, "!") ;
    }
  }

  bool FactorGraph::marginalizeStates(std::vector<StateID> States, const double Inflation)
  {
    /** time measurement */
    Timer MargTimer;

    /**check if state vector has been filled */
    if (States.empty())
    {
      PRINT_ERROR("Request for marginalization without any state.");
      return false;
    }

    std::vector<double*> MarginalStates;
    int MarginalSize = 0;
    for (const StateID &State : States)
    {
      /** check if this is a set of valid states */
      if (!this->StateData_.checkElement(State.ID, State.Timestamp, State.Number))
      {
        PRINT_ERROR("State doesn't exist at: ", State.Timestamp, " Type: ", State.ID, " Number: ", State.Number);
        return false;
      }

      /** store pointer */
      MarginalStates.push_back(
          StateData_.getElement(State.ID, State.Timestamp, State.Number).getMeanPointer());

      /** compute size of the marginalized system */
      MarginalSize += Graph_.ParameterBlockLocalSize(MarginalStates.back());
    }

    /** get connected states */
    std::vector<double*> ConnectedStates;
    std::vector<ceres::ResidualBlockId> Factors;
    std::vector<int> GlobalSize;
    std::vector<int> LocalSize;
    std::vector<StateID> StateIDs;
    std::vector<DataType> StateTypes;
    Structure_.getMarginalizationInfo(MarginalStates, ConnectedStates, Factors, GlobalSize, LocalSize, StateIDs, StateTypes);

    if (!ConnectedStates.empty())
    {
      /** calculate mean and uncertainty*/
      ceres::Problem::EvaluateOptions Options;
      Options.apply_loss_function = true;
      Options.num_threads = static_cast<int>(std::thread::hardware_concurrency());

      /** set which states should be evaluated (base + the connected ones) */
      std::vector<double*> CombinedStates;
      CombinedStates.insert(CombinedStates.end(), MarginalStates.begin(), MarginalStates.end());
      CombinedStates.insert(CombinedStates.end(), ConnectedStates.begin(), ConnectedStates.end());
      Options.parameter_blocks = CombinedStates;

      /** set which factors should be evaluated */
      Options.residual_blocks = Factors;

      /** evaluate sub-problem */
      ceres::CRSMatrix JacobianCRS;
      std::vector<double> ResidualVec;
      Graph_.Evaluate(Options, nullptr, &ResidualVec, nullptr, &JacobianCRS);

      /** map error to vector */
      Vector Residuals = Eigen::Map<Vector, Eigen::Unaligned>(ResidualVec.data(), ResidualVec.size());

      /** convert into eigen matrix */
      Matrix Jacobian;
      CRSToMatrix(JacobianCRS, Jacobian);

      /** compute marginalization */
      Matrix JacobianMarg;
      Vector ResidualMarg;
      Marginalize(Residuals, Jacobian, ResidualMarg, JacobianMarg, MarginalSize, Inflation);

      /** store original states */
      std::vector<Vector> OriginalStates;
      for (int n = 0; n < static_cast<int>(ConnectedStates.size()); n++)
      {
        VectorRef<double, Dynamic> State(ConnectedStates.at(n), GlobalSize.at(n));
        OriginalStates.emplace_back(State);
      }

      /** add factor */
      ceres::ResidualBlockId ID =
          Graph_.AddResidualBlock(new MarginalPrior(LocalSize,
                                  GlobalSize,
                                  OriginalStates,
                                  StateTypes,
                                  JacobianMarg,
                                  ResidualMarg),
                                  nullptr,
                                  ConnectedStates);

      /** add factor to internal structure */
      Structure_.addFactor<ErrorModel<0, 0>>(FactorType::Marginal, StateIDs.front().Timestamp, ID, nullptr, StateIDs, ConnectedStates, StateTypes);

      MarginalizationDuration_ += MargTimer.getSeconds();
    }
    else
    {
      PRINT_WARNING("No connected states in marginalization. Marginalized states get deleted directly!");
    }

    /** remove marginalized states in reverse order */
    std::reverse(States.begin(), States.end());
    for (const StateID &State : States)
    {
      this->removeState(State.ID, State.Timestamp, State.Number);
    }

    return true;
  }

  bool FactorGraph::marginalizeState(const string& Name, const double Timestamp, const int Number)
  {
    std::vector<StateID> SingleState;
    SingleState.emplace_back(StateID(Name, Timestamp, Number));

    return this->marginalizeStates(SingleState);
  }

  bool FactorGraph::marginalizeAllStatesOutsideWindow(const double TimeWindow, const double CurrentTime, const double Inflation)
  {
    /** calculate time boarder */
    const double CutTime = roundToTick(CurrentTime - TimeWindow);

    /** check if marginalization is required before doing anything  */
    double TimeFirst = CutTime;
    StateData_.getTimeFirstOverall(TimeFirst);
    if (TimeFirst > CutTime)
    {
      /** no marginalization required, exit here... */
      return true;
    }

    /** collect relevant states */
    std::vector<StateID> States;

    /** iterate over state names */
    const std::vector<string> StateNames = StateData_.getKeysAll();
    for (const string &Name : StateNames)
    {
      /** check if there is an element below cut time */
      double TimeFirstState;
      if (StateData_.getTimeFirst(Name, TimeFirstState))
      {
        if (TimeFirstState <= CutTime)
        {
          /** iterate over timestamps */
          std::vector<double> Times;
          if (StateData_.getTimesBelowOrEqual(Name, CutTime, Times))
          {
            for (const double Time : Times)
            {
              /** iterate over number */
              const int Numbers = StateData_.countElement(Name, Time);
              for (int n = 0; n < Numbers; n++)
              {
                States.emplace_back(StateID(Name, Time, n));
              }
            }
          }
        }
      }
    }

    /** marginalize */
    return this->marginalizeStates(States, Inflation);
  }

  bool FactorGraph::computeCovariance(const string& Name, const double Timestamp)
  {
    /** time measurement */
    Timer CovTimer;

    const bool Success = CalculateCovariance(Graph_, StateData_, Name, Timestamp);

    CovarianceDuration_ += CovTimer.getSeconds();

    return Success;
  }

  bool FactorGraph::computeCovariance(const string& Name)
  {
    /** time measurement */
    Timer CovTimer;

    const bool Success = CalculateCovariance(Graph_, StateData_, Name);

    CovarianceDuration_ += CovTimer.getSeconds();

    return Success;
  }

  bool FactorGraph::computeCovarianceSigmaPoints(const string& Name, const double Timestamp, const int StateNumber)
  {
    switch (
        StateData_.getElement(Name, Timestamp, StateNumber).getMean().size())
    {
      case 1:
        return EstimateCovarianceSigmaPoint<1>(
            Graph_, StateData_, Name, Timestamp, StateNumber);
        break;

      case 2:
        return EstimateCovarianceSigmaPoint<2>(
            Graph_, StateData_, Name, Timestamp, StateNumber);
        break;

      case 3:
        return EstimateCovarianceSigmaPoint<3>(
            Graph_, StateData_, Name, Timestamp, StateNumber);
        break;

      case 4:
        return EstimateCovarianceSigmaPoint<4>(
            Graph_, StateData_, Name, Timestamp, StateNumber);
        break;

      default:
        PRINT_ERROR("Sigma_ Point Covariance for ",
                    StateData_.getElement(Name, Timestamp, StateNumber).getMean().size(),
                    " dimensions is missing. Add case!");
        break;
    }
    return false;
  }

  void FactorGraph::sampleCost1D(const string& StateName,
                                 const double Timestamp,
                                 const int Number,
                                 const int PointCount,
                                 const double Range,
                                 StateDataSet &Result,
                                 const bool OptimizeOther)
  {
    EvaluateCostSurface<1>(
        Graph_,
        StateData_.getElement(StateName, Timestamp, Number).getMeanPointer(), PointCount, Range, Result, OptimizeOther);
  }

  void FactorGraph::sampleCost2D(const string& StateName,
                                 const double Timestamp,
                                 const int Number,
                                 const int PointCount,
                                 const double Range,
                                 StateDataSet &Result)
  {
    EvaluateCostSurface<2>(
        Graph_,
        StateData_.getElement(StateName, Timestamp, Number).getMeanPointer(), PointCount, Range, Result);
  }

  void FactorGraph::sampleCost3D(const string& StateName,
                                 const double Timestamp,
                                 const int Number,
                                 const int PointCount,
                                 const double Range,
                                 StateDataSet &Result)
  {
    EvaluateCostSurface<3>(
        Graph_,
        StateData_.getElement(StateName, Timestamp, Number).getMeanPointer(), PointCount, Range, Result);
  }

  StateDataSet &FactorGraph::getStateData()
  {
    return StateData_;
  }

  void FactorGraph::removeState(const string& Name, double Timestamp, int Number)
  {
    /** safety check */
    if (StateData_.checkElement(Name, Timestamp, Number))
    {
      /** remove from internal lists*/
      StateID State(Name, Timestamp, Number);
      Structure_.removeState(State);

      /** remove from ceres */
      Graph_.RemoveParameterBlock(
          StateData_.getElement(Name, Timestamp, Number).getMeanPointer());

      /** remove from our StateDataSet */
      StateData_.removeElement(Name, Timestamp, Number);
    }
    else
    {
      PRINT_ERROR("State doesn't exist at: ", Timestamp, " Type: ", Name, " Number: ", Number);
    }
  }

  void FactorGraph::removeState(const string& Name, const double Timestamp)
  {
    if (StateData_.checkElement(Name, Timestamp))
    {
      /** remove from ceres::problem */
      for (int StateNumber = StateData_.countElement(Name, Timestamp); StateNumber > 0; --StateNumber)
      {
        /** remove from internal lists*/
        StateID State(Name, Timestamp, StateNumber - 1);
        Structure_.removeState(State);

        /** remove from ceres */
        Graph_.RemoveParameterBlock(
            StateData_.getElement(Name, Timestamp, StateNumber - 1).getMeanPointer());

        /** remove from our StateDataSet */
        StateData_.removeElement(Name, Timestamp, StateNumber - 1);
      }
    }
    else
    {
      PRINT_ERROR("State doesn't exist at: ", Timestamp, " Type: ", Name);
    }
  }

  void FactorGraph::removeStatesOutsideWindow(const string& Name, const double TimeWindow, const double CurrentTime)
  {
    const double CutTime = CurrentTime - TimeWindow;
    double Timestamp;
    bool TimestampExists;

    /** loop over timestamps */
    TimestampExists = StateData_.getTimeFirst(Name, Timestamp);
    if (TimestampExists)
    {
      while ((Timestamp <= CutTime) && TimestampExists)
      {
        double TimestampTemp = Timestamp;
        TimestampExists = StateData_.getTimeNext(Name, Timestamp, Timestamp);
        removeState(Name, TimestampTemp);
      }
    }
    else
    {
      PRINT_ERROR("State Type doesn't exist: ", Name);
    }
  }

  void FactorGraph::removeAllStatesOutsideWindow(const double TimeWindow, const double CurrentTime)
  {
    for (auto const &State : StateData_)
    {
      removeStatesOutsideWindow(State.first, TimeWindow, CurrentTime);
    }
  }

  void FactorGraph::removeFactor(const FactorType CurrentFactorType, const double Timestamp)
  {
    if (Structure_.checkFactor(CurrentFactorType, Timestamp))
    {
      /** loop over factors */
      ceres::ResidualBlockId CeresID;
      for (int FactorNumber =
               Structure_.countFactor(CurrentFactorType, Timestamp); FactorNumber > 0; FactorNumber--)
      {
        FactorID Factor(CurrentFactorType, Timestamp, FactorNumber - 1);
        Structure_.getResidualID(Factor, CeresID);

        /** remove factor in ceres */
        Graph_.RemoveResidualBlock(CeresID);

        /** remove factor in libRSF */
        Structure_.removeFactor(Factor);
      }
    }
    else
    {
      PRINT_ERROR("Factor doesn't exist: ", CurrentFactorType, " at ", Timestamp);
    }
  }

  void FactorGraph::removeFactorsOutsideWindow(const FactorType CurrentFactorType, const double TimeWindow, const double CurrentTime)
  {
    /** find start of the existing factors */
    double FirstTime;
    Structure_.getTimeFirst(CurrentFactorType, FirstTime);

    /** calculate cut-off time */
    const double TimeWindowEnd = roundToTick(CurrentTime - TimeWindow);

    /** remove factors one by one */
    if (TimeWindowEnd >= FirstTime)
    {
      /** get relevant Timestamps */
      std::vector<double> Timestamps;
      Structure_.getTimesBelow(CurrentFactorType, TimeWindowEnd, Timestamps);

      /** remove all */
      for (const double &Time : Timestamps)
      {
        this->removeFactor(CurrentFactorType, Time);
      }
    }
  }

  void FactorGraph::removeAllFactorsOutsideWindow(const double TimeWindow, const double CurrentTime)
  {
    std::vector<FactorType> Factors;
    Structure_.getFactorTypes(Factors);
    for (auto const &Factor : Factors)
    {
      removeFactorsOutsideWindow(Factor, TimeWindow, CurrentTime);
    }
  }

  void FactorGraph::setConstantOutsideWindow(const string& Name, const double TimeWindow, const double CurrentTime)
  {
    /** find start of the current state */
    double Timestamp;
    bool TimestampExists = StateData_.getTimeFirst(Name, Timestamp);

    /** estimate end */
    const double CutTime = CurrentTime - TimeWindow;

    /** loop over timestamps that are older than CutTime */
    if (TimestampExists)
    {
      while ((Timestamp <= CutTime) && TimestampExists)
      {
        setConstant(Name, Timestamp);
        TimestampExists = StateData_.getTimeNext(Name, Timestamp, Timestamp);
      }
    }
    else if (!StateData_.checkID(Name))
    {
      PRINT_ERROR("State doesn't exist: ", Name);
    }
  }

  void FactorGraph::setVariableInsideWindow(const string& Name, const double TimeWindow, const double CurrentTime)
  {
    /** find end of the current state */
    double Timestamp;
    bool TimestampExists = StateData_.getTimeLast(Name, Timestamp);

    /** estimate end */
    const double CutTime = CurrentTime - TimeWindow;

    /** loop over timestamps that are newer than CutTime */
    if (TimestampExists)
    {
      while ((Timestamp > CutTime) && TimestampExists)
      {
        setVariable(Name, Timestamp);
        TimestampExists = StateData_.getTimePrev(Name, Timestamp, Timestamp);
      }
    }
    else if (!StateData_.checkID(Name))
    {
      PRINT_ERROR("State doesn't exist: ", Name);
    }
  }

  void FactorGraph::setAllVariableInsideWindow(const double TimeWindow, const double CurrentTime)
  {
    for (auto const &State : StateData_)
    {
      setVariableInsideWindow(State.first, TimeWindow, CurrentTime);
    }
  }

  void FactorGraph::setAllConstantOutsideWindow(const double TimeWindow, const double CurrentTime)
  {
    for (auto const &State : StateData_)
    {
      setConstantOutsideWindow(State.first, TimeWindow, CurrentTime);
    }
  }

  void FactorGraph::setVariable(const string& Name)
  {
    /** find end of the current state */
    double Timestamp;
    bool TimestampExists = StateData_.getTimeLast(Name, Timestamp);

    /** loop over all timestamps */
    if (TimestampExists)
    {
      while (TimestampExists)
      {
        setVariable(Name, Timestamp);
        TimestampExists = StateData_.getTimePrev(Name, Timestamp, Timestamp);
      }
    }
    else if (!StateData_.checkID(Name))
    {
      PRINT_ERROR("State doesn't exist: ", Name);
    }
  }

  void FactorGraph::setAllVariable()
  {
    /** loop over all state names */
    for (auto const &State : StateData_)
    {
      setVariable(State.first);
    }
  }

  double FactorGraph::getSolverDurationAndReset()
  {
    /** reset solver duration before value is returned */
    const double Duration = SolverDuration_;
    SolverDuration_ = 0.0;
    return Duration;
  }

  double FactorGraph::getCovarianceDurationAndReset()
  {
    /** reset marginalization duration before value is returned */
    const double Duration = CovarianceDuration_;
    CovarianceDuration_ = 0.0;
    return Duration;
  }


  double FactorGraph::getMarginalDurationAndReset()
  {
    /** reset marginalization duration before value is returned */
    const double Duration = MarginalizationDuration_;
    MarginalizationDuration_ = 0.0;
    return Duration;
  }

  int FactorGraph::getSolverIterationsAndReset()
  {
    /** reset marginalization duration before value is returned */
    const int Iterations = SolverIterations_;
    SolverIterations_ = 0;
    return Iterations;
  }

  void FactorGraph::enableErrorModel(FactorType CurrentFactorType)
  {
    /** loop over all models */
    std::vector<ErrorModelBase*> ErrorModels;
    Structure_.getErrorModels(CurrentFactorType, ErrorModels);
    for (const auto &ErrorModel : ErrorModels)
    {
      ErrorModel->enable();
    }
  }

  void FactorGraph::disableErrorModel(FactorType CurrentFactorType)
  {
    std::vector<ErrorModelBase*> ErrorModels;
    Structure_.getErrorModels(CurrentFactorType, ErrorModels);
    for (const auto &ErrorModel : ErrorModels)
    {
      ErrorModel->disable();
    }
  }

  void FactorGraph::enableErrorModels()
  {
    std::vector<FactorType> Factors;
    Structure_.getFactorTypes(Factors);
    for (const auto &Factor : Factors)
    {
      this->enableErrorModel(Factor);
    }
  }

  void FactorGraph::disableErrorModels()
  {
    std::vector<FactorType> Factors;
    Structure_.getFactorTypes(Factors);
    for (const auto &Factor : Factors)
    {
      this->disableErrorModel(Factor);
    }
  }

  void FactorGraph::getFactorsOfState(const string& Name, const double Timestamp, const int Number, std::vector<FactorID> &Factors) const
  {
    StateID State(Name, Timestamp, Number);
    Structure_.getFactorsOfState(State, Factors);
  }

  int FactorGraph::countFactorsOfType(const FactorType CurrentFactorType) const
  {
    return Structure_.countFactorType(CurrentFactorType);
  }

  double FactorGraph::getCost()
  {
    double Cost = 0.0;
    Graph_.Evaluate(ceres::Problem::EvaluateOptions(), &Cost, nullptr, nullptr, nullptr);
    return Cost;
  }

  void FactorGraph::computeUnweightedError(const FactorType CurrentFactorType, std::vector<double> &ErrorData)
  {
    /** get residual IDs */
    std::vector<ceres::ResidualBlockId> IDs;
    Structure_.getResidualIDs(CurrentFactorType, IDs);

    /** terminate here, if factors are missing */
    if (IDs.empty())
    {
      PRINT_WARNING("Factors of type ", CurrentFactorType, " are missing!");
      return;
    }

    /** check IDs */
    std::vector<ceres::ResidualBlockId> IDsCeres;
    std::vector<bool> Existence;
    Graph_.GetResidualBlocks(&IDsCeres);
    for (auto const &ID1 : IDs)
    {
      bool Exists = false;
      for (auto const &ID2 : IDsCeres)
      {
        if (ID1 == ID2)
        {
          Exists = true;
        }
      }
      Existence.push_back(Exists);
      if (!Exists)
      {
        PRINT_ERROR("Found missing ID");
      }
    }

    /** configure evaluation */
    ceres::Problem::EvaluateOptions Options;
    Options.num_threads = static_cast<int>(std::thread::hardware_concurrency());
    Options.residual_blocks = IDs;
    Options.apply_loss_function = false;

    /** disable error model during evaluation */
    disableErrorModel(CurrentFactorType);

    /** compute the errors */
    Graph_.Evaluate(Options, nullptr, &ErrorData, nullptr, nullptr);

    /** re-enable error model */
    enableErrorModel(CurrentFactorType);

    /** create factor ID */
    double TimeFirst;
    Structure_.getTimeFirst(CurrentFactorType, TimeFirst);
    FactorID FirstID(CurrentFactorType, TimeFirst, 0);

    /** remove unused dimensions */
    int InputSize;
    int OutputSize;
    Structure_.getErrorInputSize(FirstID, InputSize);
    Structure_.getErrorOutputSize(FirstID, OutputSize);

    for (int n = static_cast<int>(ErrorData.size()) - 1; n > 0; n -= OutputSize)
    {
      for (int m = 0 ; m < (OutputSize - InputSize); m++)
      {
        ErrorData.erase(ErrorData.begin() + n - m);
      }
    }
  }

  void FactorGraph::computeUnweightedErrorMatrix(const FactorType CurrentFactorType, Matrix &ErrorMatrix)
  {
    /** get the data */
    std::vector<double> ErrorVector;
    this->computeUnweightedError(CurrentFactorType, ErrorVector);

    /** check for empty vector */
    if (ErrorVector.empty())
    {
      PRINT_WARNING("Factors of type ", CurrentFactorType, " are missing!");
      return;
    }

    /** get the factor IDs */
    std::vector<FactorID> FactorVector;
    Structure_.getFactorIDs(CurrentFactorType, FactorVector);

    /** get the dimensions */
    const int Dim = static_cast<int>(ErrorVector.size() / FactorVector.size());
    const int Length = static_cast<int>(FactorVector.size());

    /** map std vector to matrix */
    ErrorMatrix = Eigen::Map<Matrix, Eigen::Unaligned, Eigen::Stride<1, Dynamic>>(ErrorVector.data(), Dim, Length, Eigen::Stride<1,Dynamic>(1, Dim));
  }

  void FactorGraph::computeUnweightedError(const FactorType CurrentFactorType, const string &Name, StateDataSet &ErrorData)
  {
    /** get the data */
    std::vector<double> ErrorVector;
    this->computeUnweightedError(CurrentFactorType, ErrorVector);

    /** check for empty vector */
    if (ErrorVector.empty())
    {
      PRINT_WARNING("Factors of type ", CurrentFactorType, " are missing!");
      return;
    }

    /** get the factor IDs */
    std::vector<FactorID> FactorVector;
    Structure_.getFactorIDs(CurrentFactorType, FactorVector);

    /** get the dimensions */
    const int Dim = static_cast<int>(ErrorVector.size() / FactorVector.size());
    const int Length = static_cast<int>(FactorVector.size());

    /** convert to Data */
    switch (Dim)
    {
      case 1:
        {
          Data ErrorState(DataType::Error1, 0.0);
          for (int i = 0; i < Length; i++)
          {
            const int Index = i*Dim;
            ErrorState.setTimestamp(FactorVector.at(i).Timestamp);
            ErrorState.setMean((Vector1() << ErrorVector.at(Index)).finished());
            ErrorData.addElement(Name, ErrorState);
          }
        }
        break;

      case 2:
        {
          Data ErrorState(DataType::Error2, 0.0);
          for (int i = 0; i < Length; i++)
          {
            const int Index = i*Dim;
            ErrorState.setTimestamp(FactorVector.at(i).Timestamp);
            ErrorState.setMean((Vector2() << ErrorVector.at(Index), ErrorVector.at(Index+1)).finished());
            ErrorData.addElement(Name, ErrorState);
          }
        }
        break;

      case 3:
        {
          Data ErrorState(DataType::Error3, 0.0);
          for (int i = 0; i < Length; i++)
          {
            const int Index = i*Dim;
            ErrorState.setTimestamp(FactorVector.at(i).Timestamp);
            ErrorState.setMean((Vector3() << ErrorVector.at(Index), ErrorVector.at(Index+1), ErrorVector.at(Index+2)).finished());
            ErrorData.addElement(Name, ErrorState);
          }
        }
        break;

      case 6:
        {
          Data ErrorState(DataType::Error6, 0.0);
          for (int i = 0; i < Length; i++)
          {
            const int Index = i*Dim;
            ErrorState.setTimestamp(FactorVector.at(i).Timestamp);
            ErrorState.setMean((Vector6() << ErrorVector.at(Index), ErrorVector.at(Index+1), ErrorVector.at(Index+2),
                                             ErrorVector.at(Index+3), ErrorVector.at(Index+4), ErrorVector.at(Index+5)).finished());
            ErrorData.addElement(Name, ErrorState);
          }
        }
        break;

      default:
        PRINT_ERROR("There is no Data type for errors with dimension of: ", Dim);
        break;
    }
  }

  void FactorGraph::computeUnweightedError(const FactorType CurrentFactorType, const double Time, const int Number, Vector &Error)
  {

    /** check */
    if (!Structure_.checkFactor(CurrentFactorType, Time, Number))
    {
      PRINT_ERROR("Factor does not exist: ", CurrentFactorType, " ", Time, " ", Number);
      return;
    }

    /** get internal ceres ID */
    ceres::ResidualBlockId CeresID;
    FactorID OurID(CurrentFactorType, Time, Number);
    Structure_.getResidualID(OurID, CeresID);

    std::vector<ceres::ResidualBlockId> CeresIDs;
    CeresIDs.push_back(CeresID);

    /** configure evaluation */
    ceres::Problem::EvaluateOptions Options;
    Options.num_threads = 1;
    Options.residual_blocks = CeresIDs;
    Options.apply_loss_function = false;

    /** disable error model */
    disableErrorModel(CurrentFactorType);

    /** compute error */
    std::vector<double> ErrorData;
    Graph_.Evaluate(Options, nullptr, &ErrorData, nullptr, nullptr);

    /** re-enable error model again */
    enableErrorModel(CurrentFactorType);

    /** copy to eigen vector */
    int Size;
    Structure_.getErrorInputSize(OurID, Size);
    Error.resize(Size);
    for (int n = 0; n < Size; n++)
    {
      Error(n) = ErrorData.at(n);
    }
  }

    void FactorGraph::setSolverOptions(const ceres::Solver::Options &Options)
    {
      SolverOptions_ = Options;
    }
    ceres::Solver::Options FactorGraph::getSolverOptions() const
    {
      return SolverOptions_;
    }
  }
