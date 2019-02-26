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
 * @file FactorGraphConfig.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Class that represents the configuration of the optimization problem.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORGRAPHCONFIG_H
#define FACTORGRAPHCONFIG_H

#include <ceres/ceres.h>
#include "FactorGraph.h"

/** Range Error Model */
#define ROBUST_NONE   0
#define ROBUST_DCS    1
#define ROBUST_CDCE   2
#define ROBUST_MM     5
#define ROBUST_SM     6
#define ROBUST_STSM   9
#define ROBUST_STMM   10
#define RANGE_MM_GR   12
#define RANGE_SM_GR   13
#define RANGE_STMM_GR 14
#define RANGE_STSM_GR 15
#define RANGE_STSM_VBI  16
#define RANGE_STMM_VBI  17

namespace libRSF
{
  enum class ErrorModelType {None,
                             Gaussian,
                             DCE, cDCE,
                             SC, DCS,
                             MM_GMM, SM_GMM,
                             MM_GMM_ST, SM_GMM_ST,
                             MM_GRMM, SM_GRMM,
                             MM_GRMM_ST, SM_GRMM_ST,
                             MM_GMM_SNR, SM_GMM_SNR};

  enum class SolutionType {None, Batch, Incremental};
  enum class InitializationType {Zero, Random, Transition, Measurement, GroundTruth};

  class FactorGraphConfig
  {
  public:

    FactorGraphConfig();

    /** parse the command line Options */
    bool ReadCommandLineOptions(int argc, char** argv);

    struct RangeErrorModel
    {
      uint32_t Type;
    } RangeErrorModel;

    char* InputFile;
    char* OutputFile;
  };
}

#endif // FACTORGRAPHCONFIG_H
