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


#include "AppPool_Adaptive.h"


bool AdaptErrorModel(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     libRSF::Data &IterationSummary)
{
  bool HasAdapted = false;

  /** measure runtime */
  libRSF::Timer AdaptionTimer;

  if(Config.GNSS.IsActive)
  {
    if(Config.GNSS.ErrorModel.Type == libRSF::ErrorModelType::GMM)
    {
      if (Config.GNSS.ErrorModel.GMM.TuningType != libRSF::ErrorModelTuningType::None)
      {
        AdaptGeneric1D(Graph, Config.GNSS.Type, Config.GNSS.ErrorModel, true, false);
        HasAdapted = true;
      }
    }
  }

  if(Config.Ranging.IsActive)
  {
    if(Config.Ranging.ErrorModel.Type == libRSF::ErrorModelType::GMM)
    {
      if (Config.Ranging.ErrorModel.GMM.TuningType != libRSF::ErrorModelTuningType::None)
      {
        AdaptGeneric1D(Graph, Config.Ranging.Type, Config.Ranging.ErrorModel, false, false);
        HasAdapted = true;
      }
    }
  }

  if(Config.LoopClosure.IsActive)
  {
    if(Config.LoopClosure.ErrorModel.Type == libRSF::ErrorModelType::GMM)
    {
      if (Config.LoopClosure.ErrorModel.GMM.TuningType != libRSF::ErrorModelTuningType::None)
      {
        switch (Config.LoopClosure.Type)
        {
          case libRSF::FactorType::LoopPose2:
            AdaptGeneric<3>(Graph, Config.LoopClosure.Type, Config.LoopClosure.ErrorModel, true);
            break;

          case libRSF::FactorType::Loop2:
            AdaptGeneric<2>(Graph, Config.LoopClosure.Type, Config.LoopClosure.ErrorModel, true);
            break;

          default:
            PRINT_ERROR("Unknown LoopClosure Type: ", Config.LoopClosure.Type);
            break;
        }
        HasAdapted = true;
      }
    }
  }

  /** save runtime */
  if (HasAdapted)
  {
    libRSF::Vector1 Duration = IterationSummary.getValue(libRSF::DataElement::DurationAdaptive);
    Duration(0) += AdaptionTimer.getSeconds();
    IterationSummary.setValue(libRSF::DataElement::DurationAdaptive, Duration);
  }

  return HasAdapted;
}

void AdaptGeneric1D(libRSF::FactorGraph &Graph,
                  const libRSF::FactorType Factor,
                  const libRSF::FactorGraphConfig::ErrorModelConfig &ErrorConfig,
                  const bool RemoveOffset,
                  const bool UseAsymmetricInit)
{
  /** calculate error */
  libRSF::Matrix Error;
  Graph.computeUnweightedErrorMatrix(Factor, Error);

  /** init model */
  static libRSF::GaussianMixture<1> GMM;
  if (ErrorConfig.GMM.IncrementalTuning && Error.size() > 0)
  {
      /** add first component if empty */
      if(GMM.getNumberOfComponents() == 0)
      {
          GMM.initSpread(1, ErrorConfig.GMM.BaseStandardDeviation);
      }

      /** remove smallest component if limit exceeded */
      if(GMM.getNumberOfComponents() >= 10)
      {
        GMM.sortComponentsByWeight();
        GMM.removeLastComponent();
      }

      /** calculate statistics for GMM initialization*/
      const libRSF::Vector1 Mean = -Error.rowwise().mean();
      const libRSF::Vector1 Covariance = libRSF::EstimateSampleCovariance<1>(Error);
      const libRSF::Vector1 Weight = libRSF::Vector1::Ones() / GMM.getNumberOfComponents();

      /** add just one component per timestamp */
      libRSF::GaussianComponent<1> Component;
      Component.setParamsCovariance(Covariance, Mean, Weight);

      GMM.addComponent(Component);
  }
  else
  {
    /** initialize with a simple default model */
    GMM.initSpread(ErrorConfig.GMM.NumberComponents, ErrorConfig.GMM.BaseStandardDeviation, UseAsymmetricInit);
  }

  /** set GMM estimation config */
  libRSF::GaussianMixture<1>::EstimationConfig Config = libRSF::GaussianMixture<1>::ConvertConfig(ErrorConfig.GMM);
  Config.MergeSimilarComponents = false;

  /** adapt error model */
  if(!GMM.estimate(Error, Config))
  {
    return;
  }

  /** try to set main component to zero */
  if(RemoveOffset)
  {
    GMM.removeOffset();
  }

  /** replace model */
  switch (ErrorConfig.GMM.MixtureType)
  {
  case libRSF::ErrorModelMixtureType::MaxMix:
    {
      const libRSF::MaxMix1 NewModel(GMM);
      Graph.setNewErrorModel(Factor, NewModel);
    }
    break;

  case libRSF::ErrorModelMixtureType::SumMix:
    {
      const libRSF::SumMix1 NewModel(GMM);
      Graph.setNewErrorModel(Factor, NewModel);
    }
    break;

  case libRSF::ErrorModelMixtureType::MaxSumMix:
    {
      const libRSF::MaxSumMix1 NewModel(GMM);
      Graph.setNewErrorModel(Factor, NewModel);
    }
    break;

  default:
    PRINT_ERROR("Wrong mixture type!");
      break;
  }
}
