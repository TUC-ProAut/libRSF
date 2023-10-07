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
 * @file Example_EM_1D.cpp
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A simple example application to verify algorithms that estimate the parameters of a GMM.
 * @copyright GNU Public License.
 *
 */

#include "Example_GMM_Estimation.h"

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** define distribution parameter */
//  int N = 10000000; //original 11s (only EM)
  const int N = 1000;

  /** define component 1 */
  const libRSF::Vector1 Mean1 = libRSF::Vector1::Zero();
  const libRSF::Matrix11 Cov1 = libRSF::Matrix11::Identity() * 5*5;
  const libRSF::Vector1 Weight1 = libRSF::Vector1::Ones() * 0.75;

  /** define component 2 */
  const libRSF::Vector1 Mean2 = libRSF::Vector1::Ones() * 100;
  const libRSF::Matrix11 Cov2 = libRSF::Matrix11::Identity() * 50*50;
  const libRSF::Vector1 Weight2= libRSF::Vector1::Ones() - Weight1;

  /** define basic estimation parameter */
  const int NumberOfComponents = 2;
  const double BaseStd = 10.0;

  /** sample from GMM */
  const libRSF::Vector Data = GenerateSamplesGMM<1>(N,
                                                    -Mean1,Cov1,Weight1,
                                                    -Mean2,Cov2,Weight2);
  /** initialize GMM */
  libRSF::GaussianMixture<1> GMM_true;
  libRSF::GaussianMixture<1> GMM_EM;
  libRSF::GaussianMixture<1> GMM_EM_MAP;
  libRSF::GaussianMixture<1> GMM_VBI;
  libRSF::GaussianMixture<1> GMM_VBI_Full;
  libRSF::GaussianComponent<1> Component1;
  libRSF::GaussianComponent<1> Component2;

  /** initialize with true values */
  Component1.setParamsCovariance(Cov1, Mean1, Weight1);
  Component2.setParamsCovariance(Cov2, Mean2, Weight2);
  GMM_true.addComponent(Component1);
  GMM_true.addComponent(Component2);

  /** initialize with wrong values */
  GMM_EM.initSpread(NumberOfComponents, BaseStd);
  GMM_EM_MAP.initSpread(NumberOfComponents, BaseStd);
  GMM_VBI.initSpread(NumberOfComponents, BaseStd);
  GMM_VBI_Full.initSpread(NumberOfComponents, BaseStd);

  /** set config */
  libRSF::GaussianMixture<1>::EstimationConfig ConfigEM;
  libRSF::GaussianMixture<1>::EstimationConfig ConfigEM_MAP;
  libRSF::GaussianMixture<1>::EstimationConfig ConfigVBI;
  libRSF::GaussianMixture<1>::EstimationConfig ConfigVBI_Full;

  ConfigEM.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM;
  ConfigEM.MaxIterations = 200;
  ConfigEM.MinLikelihoodChange = 1e-8;
  ConfigEM.RemoveSmallComponents = true;
  ConfigEM.MinimalSamples = std::min(N,15);
  ConfigEM.MinSamplePerComponent = ConfigEM.MinimalSamples/NumberOfComponents;

  ConfigEM_MAP = ConfigEM;
  ConfigEM_MAP.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM_MAP;
  ConfigEM_MAP.PriorDirichletConcentration = 1.0;

  ConfigVBI = ConfigEM_MAP;
  ConfigVBI.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI;

  ConfigVBI_Full = ConfigEM_MAP;
  ConfigVBI_Full.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI_Full;

  /** fit mixture to data */
  libRSF::Timer Timer;
  GMM_EM.estimate(Data, ConfigEM);
  const double TimeEM = Timer.getMilliseconds();

  Timer.reset();
  GMM_EM_MAP.estimate(Data, ConfigEM_MAP);
  const double TimeEM_MAP = Timer.getMilliseconds();

  Timer.reset();
  GMM_VBI.estimate(Data, ConfigVBI);
  const double TimeVBI = Timer.getMilliseconds();

  Timer.reset();
  GMM_VBI_Full.estimate(Data, ConfigVBI_Full);
  const double TimeVBI_Full = Timer.getMilliseconds();

  /** check fitted GMM */
  std::cout << std::endl;
  std::cout << "True Parameters:" << std::endl;
  GMM_true.printParameter();
  std::cout << std::endl;

  std::cout << "Estimated Parameters EM:" << std::endl;
  GMM_EM.printParameter();
  std::cout << "Runtime: " << TimeEM << " ms" << std::endl;
  std::cout << std::endl;

  std::cout << "Estimated Parameters EM-MAP:" << std::endl;
  GMM_EM_MAP.printParameter();
  std::cout << "Runtime: " << TimeEM_MAP << " ms" << std::endl;
  std::cout << std::endl;

  std::cout << "Estimated Parameters VBI:" << std::endl;
  GMM_VBI.printParameter();
  std::cout << "Runtime: " << TimeVBI << " ms" << std::endl;
  std::cout << std::endl;

  std::cout << "Estimated Parameters VBI-Full:" << std::endl;
  GMM_VBI_Full.printParameter();
  std::cout << "Runtime: " << TimeVBI_Full << " ms" << std::endl;

  return 0;
}
