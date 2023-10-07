/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
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
 * @file Test_App_Robust_Models_2D.cpp
 * @author Leopold Mauersberger
 * @date 18 Mar 2021
 * @brief Comparing the App_Robust_Models_2D results against sample solution
 * @copyright GNU Public License.
 *
 */

#include "../applications/App_Robust_Models_2D.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

void App_Robust_Models_2D_Testfunction(std::string Model, const double Mean, const double MaxAllowedError)
{
/** configure arguments */
  const int NumPointsPerDim = 10;

  /** set command line arguments */
  const std::vector<std::string> ArgumentsIn{"App_Test","empty", "empty", "Data_2D_Output.txt", std::to_string(NumPointsPerDim), "8", Model,
                                             std::to_string(Mean), std::to_string(Mean), std::to_string(Mean), std::to_string(Mean),
                                             "0.5", "0", "0", "1", "2", "0", "0", "5", "0.35", "0.65"};

  /** convert to char pointers for a consistent interface */
  std::vector<char*> ArgV;
  for (const auto& arg : ArgumentsIn)
  {
    ArgV.push_back((char*)arg.data());
  }
  ArgV.push_back(nullptr);

  /** pass through the config parser */
  std::vector<std::string> Arguments;
  libRSF::FactorGraphConfig Config;
  Config.ReadCommandLineOptions(ArgV.size() - 1, ArgV.data(), &Arguments);

  /** data sets that store results*/
  libRSF::StateDataSet CostSurfaceData;
  libRSF::StateDataSet PreOptimizationData;
  libRSF::StateDataSet PostOptimizationData;
  libRSF::StateDataSet SolverData;

  ASSERT_FALSE(CreateGraphAndSolve(Arguments,
                                   CostSurfaceData, PreOptimizationData, PostOptimizationData,
                                   SolverData)) << "Error calculating example";

  /** create expected result */
  libRSF::Vector2 MeanVector;
  MeanVector.fill(-Mean);

  libRSF::Data GT(libRSF::DataType::Point2, 0.0);
  GT.setMean(MeanVector);

  libRSF::SensorDataSet Expected;
  for (int nPose = 0; nPose < NumPointsPerDim*NumPointsPerDim; nPose++)
  {
      Expected.addElement(GT);
  }

  const double MaxAbsErrorMean = libRSF::MaxAbsError(libRSF::DataType::Point2,
                                                     Expected,
                                                     POSITION_STATE,
                                                     PostOptimizationData,
                                                     libRSF::DataElement::Mean);
  std::cout << "Maximum absolute derivation: " << MaxAbsErrorMean << std::endl;

  EXPECT_LT(MaxAbsErrorMean, MaxAllowedError);

}

TEST(App_Robust_Models_2D, MaxSumMix_0)
{
  App_Robust_Models_2D_Testfunction("MaxSumMix", 0.0, 0.01);
}

TEST(App_Robust_Models_2D, MaxSumMix_1)
{
  App_Robust_Models_2D_Testfunction("MaxSumMix", 1.0, 0.01);
}

TEST(App_Robust_Models_2D, Gaussian_0)
{
  App_Robust_Models_2D_Testfunction("Gaussian", 0.0, 0.01);
}

/* Gaussian ignores mean
TEST(App_Robust_Models_2D, Gaussian_1)
{
  App_Robust_Models_2D_Testfunction("Gaussian", 1.0, 0.01);
}
*/

TEST(App_Robust_Models_2D, MaxMix_0)
{
  App_Robust_Models_2D_Testfunction("MaxMix", 0.0, 0.01);
}

TEST(App_Robust_Models_2D, MaxMix_1)
{
  App_Robust_Models_2D_Testfunction("MaxMix", 1.0, 0.01);
}

/* will not work
TEST(App_Robust_Models_2D, SumMix_0)
{
  App_Robust_Models_2D_Testfunction("SumMix", 0.0, 0.01);
}

TEST(App_Robust_Models_2D, SumMix_1)
{
  App_Robust_Models_2D_Testfunction("SumMix", 1.0, 0.01);
}
*/

TEST(App_Robust_Models_2D, DCS_0)
{
  App_Robust_Models_2D_Testfunction("DCS", 0.0, 0.01);
}

/* DCS ignores mean
TEST(App_Robust_Models_2D, DCS_1)
{
  App_Robust_Models_2D_Testfunction("DCS", 1.0, 0.01);
}
*/

// main provided by linking to gtest_main
