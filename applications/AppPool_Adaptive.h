/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option); any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de);
 ***************************************************************************/

/**
 * @file AppPool_Adaptive.h
 * @author Tim Pfeifer
 * @date 09 August 2019
 * @brief Contains a set of functions, related to adaptive error models.
 * @copyright GNU Public License.
 *
 */

#ifndef APPPOOL_ADAPTIVE_H
#define APPPOOL_ADAPTIVE_H

#include "libRSF.h"

bool AdaptErrorModel(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     libRSF::Data &IterationSummary);

void AdaptGeneric1D(libRSF::FactorGraph &Graph,
                    libRSF::FactorType Factor,
                    const libRSF::FactorGraphConfig::ErrorModelConfig &ErrorConfig,
                    bool RemoveOffset,
                    bool UseAsymmetricInit);

template <int Dim>
void AdaptGeneric(libRSF::FactorGraph &Graph,
                  libRSF::FactorType Factor,
                  const libRSF::FactorGraphConfig::ErrorModelConfig &ErrorConfig,
                  bool EstimateMean)
{
  /** only tune with enough samples */
  if (Graph.countFactorsOfType(Factor) < std::pow(2, Dim))
  {
    PRINT_LOGGING("Not enough samples to adapt error model for factor type ", Factor , ": ", Graph.countFactorsOfType(Factor) , " < " , std::pow(2, Dim));
    return;
  }

  /** calculate error */
  libRSF::Matrix Error;
  Graph.computeUnweightedErrorMatrix(Factor, Error);

  /** handle SE(2)*/
  if (Factor == libRSF::FactorType::LoopPose2)
  {
    /** normalize angle*/
    Error.col(2) = libRSF::NormalizeAngleVector<double, Dim>(Error.col(2));

    /** TODO: find circular mean */

    /** TODO: substract mean */
  }

  /** init model */
  static libRSF::GaussianMixture<Dim> GMM;

  if (ErrorConfig.GMM.IncrementalTuning)
  {
    /** add first component if empty */
    if(GMM.getNumberOfComponents() == 0)
    {
      GMM.initSpread(1, ErrorConfig.GMM.BaseStandardDeviation);
    }

    /** remove smallest component if limit exceeded */
    if(GMM.getNumberOfComponents() >= 8)
    {
      GMM.sortComponentsByWeight();
      GMM.removeLastComponent();
    }

    /** calculate statistics for GMM initialization*/
    const libRSF::VectorStatic<Dim> Mean = -Error.rowwise().mean();
    const libRSF::MatrixStatic<Dim,Dim> Covariance = libRSF::EstimateSampleCovariance<Dim>(Error);
    const libRSF::Vector1 Weight = libRSF::Vector1::Ones() / GMM.getNumberOfComponents();

    /** add just one component per timestamp */
    libRSF::GaussianComponent<Dim> Component;
    Component.setParamsCovariance(Covariance, Mean, Weight);

    GMM.addComponent(Component);
  }
  else
  {
    /** init with a simple default model */
    GMM.initSpread(ErrorConfig.GMM.NumberComponents, ErrorConfig.GMM.BaseStandardDeviation);
  }

  /** set GMM estimation config */
  typename libRSF::GaussianMixture<Dim>::EstimationConfig Config = libRSF::GaussianMixture<Dim>::ConvertConfig(ErrorConfig.GMM);
  Config.EstimateMean = EstimateMean;
  Config.MergeSimilarComponents = false;

  /** adapt error model */
  if(!GMM.estimate(Error, Config))
  {
    return;
  }

  /** handle SE(2) case */
  if (Factor == libRSF::FactorType::LoopPose2)
  {
    /**TODO: add circular mean*/
  }

  /** replace model */
  switch (ErrorConfig.GMM.MixtureType)
  {
    case libRSF::ErrorModelMixtureType::MaxMix:
    {
      libRSF::MaxMix<Dim> NewModel(GMM);
      Graph.setNewErrorModel(Factor, NewModel);
    }
    break;

    case libRSF::ErrorModelMixtureType::SumMix:
    {
      libRSF::SumMix<Dim> NewModel(GMM);
      Graph.setNewErrorModel(Factor, NewModel);
    }
    break;

    case libRSF::ErrorModelMixtureType::MaxSumMix:
    {
      libRSF::MaxSumMix<Dim> NewModel(GMM);
      Graph.setNewErrorModel(Factor, NewModel);
    }
    break;

    default:
      PRINT_ERROR("Wrong mixture type!");
      break;
  }
}

#endif // APPPOOL_ADAPTIVE_H
