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
 * @file FactorGraphConfig.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Class that represents the configuration of the optimization problem.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORGRAPHCONFIG_H
#define FACTORGRAPHCONFIG_H

#include "Messages.h"
#include "Types.h"

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <thread>
#include <iostream>
#include <string>
#include <initializer_list>

namespace libRSF
{

  class FactorGraphConfig
  {
  public:

    FactorGraphConfig();
    virtual ~FactorGraphConfig() = default;

    /** parse the command line Options */
    bool ReadCommandLineOptions(int argc, char** argv, std::vector<std::string> * Arguments = nullptr);
    bool ReadCommandLineOptions(const std::vector<std::string> &Arguments);

    /** parse the YAML config file with yaml-cpp */
    bool ReadYAMLOptions(const std::string& YAMLFile);

    /** interface to the outer world are a set of files*/
    std::string InputFile;
    std::string OutputFile;
    std::string ConfigFile;

    /** noise model */
    struct GaussianMixtureConfig
    {
      GaussianMixtureConfig():MixtureType(ErrorModelMixtureType::None), TuningType(ErrorModelTuningType::None) {};

      ErrorModelMixtureType MixtureType;
      ErrorModelTuningType TuningType;

      /** parameters for the fixed model */
      Vector Mean;
      Vector StdDev;
      Vector Weight;

      /** parameters for the adaptive model */
      int NumberComponents = 3;
      double BaseStandardDeviation = 1.0;
      bool IncrementalTuning = false;

      /** some of the prior parameters for adaptive estimation*/
      double PriorDirichletConcentration = 1;
      double PriorNormalInfoScaling = 1e-4;
      double PriorWishartDOF = 1;
    };

    struct ErrorModelConfig
    {
      ErrorModelType Type;
      double Parameter;

      GaussianMixtureConfig GMM; /**< only if a GMM is used */
    };

    /** factor configuration */
    struct FactorConfig
    {
      FactorType Type;
      Vector Parameter;
      ErrorModelConfig ErrorModel;

      bool IsActive{false};
    }Ranging, GNSS, ClockModel, MotionModel, Odom, IMU, Laser, Radar, Vision, LoopClosure, Tracking, Prior, Pressure;

    /** special properties */
    struct GraphConfig
    {
      SolutionType Type;

      /** available time for one iteration */
      double MaxTime; //TODO: check why unused???
      int MaxIterations;

      /** time handling */
      bool IsAsync;
      double AsyncRate;
      DataType SyncSensor;

      /** covariance estimation */
      bool EstimateCov;

      /**only for sliding window */
      bool Marginalize;
      double WindowLength;
    } Solution{};

    ceres::Solver::Options SolverConfig;

  private:
    static bool ParseErrorModelFromYAML_(YAML::Node ErrorModelNode, ErrorModelConfig &Model);
    static Vector ParseVectorFromYAML_(const YAML::Node &VectorNode);
  };
}

#endif // FACTORGRAPHCONFIG_H
