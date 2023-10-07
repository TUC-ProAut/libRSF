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
 * @file Example_EM_2D.cpp
 * @author Tim Pfeifer
 * @date 12.03.2021
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
  const int N = 1000;
  const int NumberOfComponents = 5;

  /** define component 1 */
  const libRSF::Vector2 Mean1 = libRSF::Vector2::Ones() * 0.0;
  const libRSF::Matrix22 Cov1 = libRSF::Matrix22::Identity() * 1.0;
  const libRSF::Vector1 Weight1 = libRSF::Vector1::Ones() * 0.6;

  /** define component 2 */
  const libRSF::Vector2 Mean2 = libRSF::Vector2::Ones() * 5.0;
  libRSF::Matrix22 Cov2 = libRSF::Matrix22::Identity() * 9.0;
  Cov2(1,1) = 16;
  const libRSF::Vector1 Weight2 = libRSF::Vector1::Ones() - Weight1;

  /** create data */
  const libRSF::Matrix Samples = GenerateSamplesGMM<2>(N,
                                                    -Mean1,Cov1,Weight1,
                                                    -Mean2,Cov2,Weight2);

  /** initialize GMM */
  libRSF::GaussianMixture<2> GMM_true;
  libRSF::GaussianMixture<2> GMM_EM;
  libRSF::GaussianMixture<2> GMM_EM_MAP;
  libRSF::GaussianMixture<2> GMM_VBI;
  libRSF::GaussianMixture<2> GMM_VBI_Full;
  libRSF::GaussianComponent<2> Component1;
  libRSF::GaussianComponent<2> Component2;

  /** initialize with true values */
  Component1.setParamsCovariance(Cov1, Mean1, Weight1);
  Component2.setParamsCovariance(Cov2, Mean2, Weight2);
  GMM_true.addComponent(Component1);
  GMM_true.addComponent(Component2);

  /** initialize with wrong values */
  GMM_EM.initSpread(NumberOfComponents, 10);
  GMM_EM_MAP.initSpread(NumberOfComponents, 10);
  GMM_VBI.initSpread(NumberOfComponents, 10);
  GMM_VBI_Full.initSpread(NumberOfComponents, 10);

  /** set config */
  libRSF::GaussianMixture<2>::EstimationConfig ConfigEM, ConfigEM_MAP, ConfigVBI, ConfigVBI_Full;

  ConfigEM.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM;
  ConfigEM.RemoveSmallComponents = true;

  ConfigEM_MAP.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM_MAP;
  ConfigEM_MAP.RemoveSmallComponents = true;

  ConfigVBI.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI;
  ConfigVBI.RemoveSmallComponents = true;

  ConfigVBI_Full.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI_Full;
  ConfigVBI_Full.RemoveSmallComponents = true;

  /** fit mixture to data */
  libRSF::Timer Timer;
  GMM_EM.estimate(Samples, ConfigEM);
  const double TimeEM = Timer.getMilliseconds();

  Timer.reset();
  GMM_EM_MAP.estimate(Samples, ConfigEM_MAP);
  const double TimeEM_MAP = Timer.getMilliseconds();

  Timer.reset();
  GMM_VBI.estimate(Samples, ConfigVBI);
  const double TimeVBI = Timer.getMilliseconds();

  Timer.reset();
  GMM_VBI_Full.estimate(Samples, ConfigVBI_Full);
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
