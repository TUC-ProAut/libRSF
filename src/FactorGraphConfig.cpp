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

#include "FactorGraphConfig.h"

namespace libRSF
{
  bool GetSubYAML(const std::string& Key, const YAML::Node &Node, YAML::Node &Subnode)
  {
    if(Node[Key].IsDefined())
    {
      Subnode = Node[Key];
      return true;
    }

    PRINT_ERROR("Node \"", Key, "\" is not defined!");
    return false;
  }

  Vector FactorGraphConfig::ParseVectorFromYAML_(const YAML::Node &VectorNode)
  {
    const int Length = static_cast<int>(VectorNode.size());
    Vector Vec(Length);

    if(Length == 0)
    {
      PRINT_ERROR("Empty element in YAML config: ", VectorNode.Tag());
    }

    for(int n = 0; n < Length; n++)
    {
      Vec(n) = VectorNode[n].as<double>();
    }

    return Vec;
  }

  FactorGraphConfig::FactorGraphConfig()
  {
    /** set useful default options */
    SolverConfig.minimizer_type = ceres::MinimizerType::TRUST_REGION;
    SolverConfig.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
    SolverConfig.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
    SolverConfig.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    SolverConfig.use_nonmonotonic_steps = true;
    SolverConfig.num_threads = static_cast<int>(std::thread::hardware_concurrency());
    SolverConfig.max_num_iterations = 100;
    SolverConfig.max_solver_time_in_seconds = 10.0;
    SolverConfig.minimizer_progress_to_stdout = true;

    /** debugging is simpler with one thread */
    #ifndef NDEBUG
    SolverConfig.num_threads = 1;
    #endif // NDEBUG
  }

  bool FactorGraphConfig::ParseErrorModelFromYAML_(YAML::Node ErrorModelNode, ErrorModelConfig &Model)
  {
    /** read error model type */
    auto ErrorTypeString = ErrorModelNode["type"].as<std::string>();

    if(!TranslateSafe(ErrorModelTypeDict, ErrorTypeString, Model.Type))
    {
      PRINT_ERROR("Wrong error model type: ", ErrorTypeString);
      return false;
    }

    /** read parameter depending on model type */
    switch(Model.Type)
    {
      case ErrorModelType::SC:
      case ErrorModelType::DCS:
        if (ErrorTypeString == "sc1" || ErrorTypeString == "dcs1")
        {
          /** set default parameter */
          Model.Parameter = 1.0;
        }
        else
        {
          Model.Parameter = ErrorModelNode["parameter"].as<double>();
        }
        break;

      case ErrorModelType::GMM:
        {
          auto MixtureString = ErrorModelNode["mixture_type"].as<std::string>();
          auto TuningString = ErrorModelNode["tuning_type"].as<std::string>();

          if(!TranslateSafe(ErrorModelMixtureTypeDict, MixtureString, Model.GMM.MixtureType))
          {
            PRINT_ERROR("Wrong mixture model type: ", MixtureString);
            return false;
          }

          if(!TranslateSafe(ErrorModelTuningTypeDict, TuningString, Model.GMM.TuningType))
          {
            PRINT_ERROR("Wrong tuning model type: ", TuningString);
            return false;
          }

          /** read parameter depending on tuning type */
          switch(Model.GMM.TuningType)
          {
            case ErrorModelTuningType::None:
              /** get fixed params */
              Model.GMM.Mean = ParseVectorFromYAML_(ErrorModelNode["mean"]);
              Model.GMM.StdDev = ParseVectorFromYAML_(ErrorModelNode["std_dev"]);
              Model.GMM.Weight = ParseVectorFromYAML_(ErrorModelNode["weight"]);
              break;

            case ErrorModelTuningType::EM:
              Model.GMM.NumberComponents = static_cast<int>(ErrorModelNode["components"].as<double>());
              Model.GMM.BaseStandardDeviation = ErrorModelNode["std_dev"].as<double>();
              Model.GMM.IncrementalTuning = ErrorModelNode["incremental"].as<bool>();
              break;

            case ErrorModelTuningType::EM_MAP:
              Model.GMM.NumberComponents = static_cast<int>(ErrorModelNode["components"].as<double>());
              Model.GMM.BaseStandardDeviation = ErrorModelNode["std_dev"].as<double>();
              Model.GMM.IncrementalTuning = ErrorModelNode["incremental"].as<bool>();
              /** prior parameters */
              Model.GMM.PriorDirichletConcentration = ErrorModelNode["dir_conc"].as<double>();
              Model.GMM.PriorNormalInfoScaling = ErrorModelNode["nw_scale"].as<double>();
              Model.GMM.PriorWishartDOF = ErrorModelNode["nw_dof"].as<double>();
              break;

            case ErrorModelTuningType::VBI:
              Model.GMM.NumberComponents = static_cast<int>(ErrorModelNode["components"].as<double>());
              Model.GMM.BaseStandardDeviation = ErrorModelNode["std_dev"].as<double>();
              Model.GMM.IncrementalTuning = ErrorModelNode["incremental"].as<bool>();
              /** prior parameters */
              Model.GMM.PriorNormalInfoScaling = ErrorModelNode["nw_scale"].as<double>();
              Model.GMM.PriorWishartDOF = ErrorModelNode["nw_dof"].as<double>();
              break;

            case ErrorModelTuningType::VBI_Full:
              Model.GMM.NumberComponents = static_cast<int>(ErrorModelNode["components"].as<double>());
              Model.GMM.BaseStandardDeviation = ErrorModelNode["std_dev"].as<double>();
              Model.GMM.IncrementalTuning = ErrorModelNode["incremental"].as<bool>();
              /** prior parameters */
              Model.GMM.PriorDirichletConcentration = ErrorModelNode["dir_conc"].as<double>();
              Model.GMM.PriorNormalInfoScaling = ErrorModelNode["nw_scale"].as<double>();
              Model.GMM.PriorWishartDOF = ErrorModelNode["nw_dof"].as<double>();
              break;

            default:
              PRINT_WARNING("Error model tuning type not handled: ", TuningString);
              break;
          }

        }
        break;

      default:
//        PRINT_WARNING("Error model not handled: ", ErrorTypeString);
        break;
    }

    return true;
  }

  bool FactorGraphConfig::ReadYAMLOptions(const std::string& YAMLFile)
  {
    /** check if file available */
    if (FILE *File = fopen(YAMLFile.c_str(), "r"))
    {
        fclose(File);
    }
    else
    {
      PRINT_ERROR("There is no YAML file with the name ", YAMLFile);
      return false;
    }

    /** read all files */
    const YAML::Node YAMLComplete = YAML::LoadFile(YAMLFile);

    /** config node holds the relevant configuration */
    const YAML::Node YAMLConfig  = YAMLComplete["config"];

    /** solution */
    auto SolTypeString = YAMLConfig["solution"]["solver_mode"].as<std::string>();
    if(!TranslateSafe(SolutionTypeDict, SolTypeString, Solution.Type))
    {
      PRINT_ERROR("Wrong solution type: ", SolTypeString);
      return false;
    }

    switch (Solution.Type)
    {
    case SolutionType::Batch:
        SolverConfig.minimizer_progress_to_stdout = false;

        SolverConfig.max_num_iterations = static_cast<int>(YAMLConfig["solution"]["max_iterations"].as<double>());
        SolverConfig.max_solver_time_in_seconds = YAMLConfig["solution"]["max_time"].as<double>();

        Solution.EstimateCov = YAMLConfig["solution"]["estimate_cov"].as<bool>();
        break;

    case SolutionType::Smoother:
        SolverConfig.minimizer_progress_to_stdout = false;

        SolverConfig.max_num_iterations = static_cast<int>(YAMLConfig["solution"]["max_iterations"].as<double>());
        SolverConfig.max_solver_time_in_seconds = YAMLConfig["solution"]["max_time"].as<double>();

        Solution.EstimateCov = YAMLConfig["solution"]["estimate_cov"].as<bool>();
        break;

    case SolutionType::SmootherRT:
        SolverConfig.minimizer_progress_to_stdout = false;

        SolverConfig.max_num_iterations = static_cast<int>(YAMLConfig["solution"]["max_iterations"].as<double>());
        SolverConfig.max_solver_time_in_seconds = YAMLConfig["solution"]["max_time"].as<double>();
        Solution.WindowLength = YAMLConfig["solution"]["window_length"].as<double>();

        Solution.EstimateCov = YAMLConfig["solution"]["estimate_cov"].as<bool>();
        break;

    case SolutionType::Window:
        SolverConfig.minimizer_progress_to_stdout = false;

        SolverConfig.max_num_iterations = static_cast<int>(YAMLConfig["solution"]["max_iterations"].as<double>());
        SolverConfig.max_solver_time_in_seconds = YAMLConfig["solution"]["max_time"].as<double>();

        Solution.WindowLength = YAMLConfig["solution"]["window_length"].as<double>();
        Solution.EstimateCov = YAMLConfig["solution"]["estimate_cov"].as<bool>();
        Solution.Marginalize = YAMLConfig["solution"]["marginalize"].as<bool>();
        break;

    case SolutionType::Filter:
        SolverConfig.minimizer_progress_to_stdout = false;

        /** simulate Gauss-Newtons steps using the DL step with a huge TR radius */
        SolverConfig.use_nonmonotonic_steps = true;
        SolverConfig.linear_solver_type = ceres::DENSE_QR;
        SolverConfig.initial_trust_region_radius = 1e8;
        SolverConfig.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
        SolverConfig.dogleg_type = ceres::DoglegType::TRADITIONAL_DOGLEG;
        SolverConfig.num_threads = 1;

        SolverConfig.max_num_iterations = static_cast<int>(YAMLConfig["solution"]["max_iterations"].as<double>());
        SolverConfig.max_solver_time_in_seconds = YAMLConfig["solution"]["max_time"].as<double>();

        Solution.EstimateCov = YAMLConfig["solution"]["estimate_cov"].as<bool>();
        break;

    case SolutionType::None:
        /** do nothing */
        break;

    default:
      PRINT_ERROR("Solution type not handled: ", SolTypeString);
        break;
    }

    /** get sensor synchronization strategy*/
    Solution.IsAsync = YAMLConfig["graph"]["is_async"].as<bool>();
    if(Solution.IsAsync)
    {
      Solution.AsyncRate = YAMLConfig["graph"]["async_rate"].as<double>();
    }
    else
    {
      auto SyncTypeString = YAMLConfig["graph"]["sync_sensor"]["type"].as<std::string>();
      FactorType SyncFactor;
      if(!TranslateSafe(FactorTypeDict, SyncTypeString, SyncFactor))
      {
        PRINT_ERROR("Wrong factor type: ", SyncTypeString);
        return false;
      }
      Solution.SyncSensor = FactorSensorDict.at(SyncFactor);
    }

    /** parse factors */
    for(int nFactor = 0; nFactor < static_cast<int>(YAMLConfig["factors"].size()); nFactor++)
    {
      /** create active factor */
      FactorConfig Factor;
      Factor.IsActive = true;

      /** read type */
      auto FactorTypeString = YAMLConfig["factors"][nFactor]["type"].as<std::string>();
      if(!TranslateSafe(FactorTypeDict, FactorTypeString, Factor.Type))
      {
        PRINT_ERROR("Wrong factor type: ", FactorTypeString);
        return false;
      }

      /** read error model */
      if(!ParseErrorModelFromYAML_(YAMLConfig["factors"][nFactor]["error"], Factor.ErrorModel))
      {
        PRINT_ERROR("Read error model went wrong!");
        return false;
      }

      /** read type dependent parameters and save config */
      if(TranslateSafe(FactorTypeDict, FactorTypeString, Factor.Type))
      {
        switch(Factor.Type)
        {
          case FactorType::ConstDrift1:
          case FactorType::ConstVal1:
            Factor.Parameter = ParseVectorFromYAML_(YAMLConfig["factors"][nFactor]["std_dev"]);
            break;

          case FactorType::Prior3:
            Factor.Parameter.resize(6);
            Factor.Parameter.head(3) = ParseVectorFromYAML_(YAMLConfig["factors"][nFactor]["mean"]);
            Factor.Parameter.tail(3) = ParseVectorFromYAML_(YAMLConfig["factors"][nFactor]["sqrt_info"]);
            break;

          case FactorType::IMUSimple:
            Factor.Parameter.resize(6);
            Factor.Parameter(0) = YAMLConfig["factors"][nFactor]["noise_acc"].as<double>();
            Factor.Parameter(1) = YAMLConfig["factors"][nFactor]["noise_tr"].as<double>();
            Factor.Parameter(2) = YAMLConfig["factors"][nFactor]["noise_b_acc"].as<double>();
            Factor.Parameter(3) = YAMLConfig["factors"][nFactor]["noise_b_tr"].as<double>();
            Factor.Parameter(4) = YAMLConfig["factors"][nFactor]["noise_dyn"].as<double>();
            Factor.Parameter(5) = YAMLConfig["factors"][nFactor]["interpolation_time"].as<double>();
            break;

          case FactorType::IMUPretintegration:
            Factor.Parameter.resize(4);
            Factor.Parameter(0) = YAMLConfig["factors"][nFactor]["noise_acc"].as<double>();
            Factor.Parameter(1) = YAMLConfig["factors"][nFactor]["noise_tr"].as<double>();
            Factor.Parameter(2) = YAMLConfig["factors"][nFactor]["noise_b_acc"].as<double>();
            Factor.Parameter(3) = YAMLConfig["factors"][nFactor]["noise_b_tr"].as<double>();
            break;

          case FactorType::Pseudorange3_Bias:
            Factor.Parameter.resize(1);
            Factor.Parameter(0) = YAMLConfig["factors"][nFactor]["isb_std_dev"].as<double>();
            break;

          case FactorType::Loop1:
          case FactorType::Loop2:
          case FactorType::Loop3:
            Factor.Parameter.resize(2);
            Factor.Parameter(0) = YAMLConfig["factors"][nFactor]["threshold"].as<double>();
            Factor.Parameter(1) = YAMLConfig["factors"][nFactor]["std_dev"].as<double>();
            break;

          case FactorType::LoopPose2:
            Factor.Parameter.resize(4);
            Factor.Parameter(0) = YAMLConfig["factors"][nFactor]["threshold"].as<double>();
            Factor.Parameter.tail(3) = ParseVectorFromYAML_(YAMLConfig["factors"][nFactor]["std_dev"]);
            break;


          default:
//          PRINT_WARNING("Factor not handled: ", FactorTypeString);
            break;
        }
      }
      else
      {
        PRINT_ERROR("Factor not known: ", FactorTypeString);
      }

      /** read name */
      auto FactorName = YAMLConfig["factors"][nFactor]["name"].as<std::string>();

      /** save config based on the factors type */
      AbstractFactorType FactorClass;
      if(TranslateSafe(AbstractFactorTypeDict, FactorName, FactorClass))
      {
        switch(FactorClass)
        {
        case AbstractFactorType::Prior:
          Prior = Factor;
          break;

        case AbstractFactorType::ClockModel:
          ClockModel = Factor;
          break;

        case AbstractFactorType::GNSS:
          GNSS = Factor;
          break;

        case AbstractFactorType::IMU:
          IMU = Factor;
          break;

        case AbstractFactorType::Laser:
          Laser = Factor;
          break;

        case AbstractFactorType::Odometry:
          Odom = Factor;
          break;

        case AbstractFactorType::Radar:
          Radar = Factor;
          break;

        case AbstractFactorType::Ranging:
          Ranging = Factor;
          break;

        case AbstractFactorType::Vision:
          Vision = Factor;
          break;

        case AbstractFactorType::MotionModel:
          MotionModel = Factor;
          break;

        case AbstractFactorType::LoopClosure:
          LoopClosure = Factor;
          break;

        case AbstractFactorType::Pressure:
          Pressure = Factor;
          break;

        default:
          PRINT_WARNING("Factor name not handled: ", FactorName);
          break;
        }
      }
      else
      {
        PRINT_ERROR("Factor name not known: ", FactorName);
      }
    }

    return true;
  }

  bool FactorGraphConfig::ReadCommandLineOptions(const std::vector<std::string> &Arguments)
  {
    if (Arguments.size() >= 3)
    {
      ConfigFile   = Arguments.at(0);
      InputFile    = Arguments.at(1);
      OutputFile   = Arguments.at(2);

      /** parse yaml options if there are no other options */
      if(Arguments.size() == 3)
      {
        return ReadYAMLOptions(ConfigFile);
      }
    }
    else
    {
      PRINT_WARNING("Number of Arguments doesn't match standard convention [ConfigFile InputFile OutputFile]! It has size: ", Arguments.size());
      return false;
    }
    return true;
  }

  bool FactorGraphConfig::ReadCommandLineOptions(const int argc, char** argv, std::vector<std::string> * const Arguments)
  {
    /** assign all arguments to std::string vector*/
    std::vector<std::string> Args;
    Args.assign(argv+1, argv + argc);

    /** copy arguments for passthrough*/
    if(Arguments != nullptr)
    {
      *Arguments = Args;
    }

    /** parse */
    return this->ReadCommandLineOptions(Args);
  }
}
