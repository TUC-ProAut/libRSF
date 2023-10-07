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
 * @file Test_GMM_Estimation_1D.cpp
 * @author Tim Pfeifer
 * @date 29.11.2021
 * @brief A simple test to verify algorithms that estimate the parameters of a GMM.
 * @copyright GNU Public License.
 *
 */

#include "../examples/Example_GMM_Estimation.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

TEST(Example, GMM_Estimation_1D)
{
  /** create data */
  const int N = 10000;

  /** define component 1 */
  const libRSF::Vector1 Mean1 = libRSF::Vector1::Zero();
  const libRSF::Matrix11 Cov1 = libRSF::Matrix11::Identity() * 5*5;
  const libRSF::Vector1 Weight1 = libRSF::Vector1::Ones() * 0.75;

  /** define component 2 */
  const libRSF::Vector1 Mean2 = libRSF::Vector1::Ones() * 100;
  const libRSF::Matrix11 Cov2 = libRSF::Matrix11::Identity() * 50*50;
  const libRSF::Vector1 Weight2= libRSF::Vector1::Ones() - Weight1;

  /** sample from GMM */
  const libRSF::Vector Data = GenerateSamplesGMM<1>(N,
                                                    -Mean1,Cov1,Weight1,
                                                    -Mean2,Cov2,Weight2);

  /** create configs for all algorithms */
  libRSF::GaussianMixture<1>::EstimationConfig Config;
  std::vector<libRSF::GaussianMixture<1>::EstimationConfig> Configs;

  Config.RemoveSmallComponents = true;
  Config.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM;
  Configs.push_back(Config);
  Config.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM_MAP;
  Configs.push_back(Config);
  Config.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI;
  Configs.push_back(Config);
  Config.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI_Full;
  Configs.push_back(Config);

  for(auto & Config : Configs)
  {
    /** initialize */
    libRSF::GaussianMixture<1> GMM;
    GMM.initSpread(2, 10);

    /** estimate */
    GMM.estimate(Data, Config);
    GMM.printParameter();

    /** extract components */
    std::vector<libRSF::GaussianComponent<1>> Mixture;
    GMM.getMixture(Mixture);

    /** check component 1 */
    EXPECT_LT((Mean1 - Mixture.at(0).getMean()).norm(), 0.2);
    EXPECT_LT((Cov1 - Mixture.at(0).getCovariance()).norm(), 1.5);
    EXPECT_LT((Weight1 - Mixture.at(0).getWeight()).norm(), 0.01);

    /** check component 2 (looser bound because bigger uncertainty) */
    EXPECT_LT((Mean2 - Mixture.at(1).getMean()).norm(), 4.0);
    EXPECT_LT((Cov2 - Mixture.at(1).getCovariance()).norm(), 200);
    EXPECT_LT((Weight2 - Mixture.at(1).getWeight()).norm(), 0.01);
  }
}

/** main provided by linking to gtest_main */
