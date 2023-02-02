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
 * @file Test_App_Ranging_2D.cpp
 * @author Tim Pfeifer
 * @date 02 Feb 2023
 * @brief File containing a test for an application that estimates a 2D pose based on range measurements.
 * @copyright GNU Public License.
 *
 */

#include "../applications/App_Ranging_2D.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

/** define paths */
#define CONFIG "config/Default_Ranging.yaml"
#define DATA_M_HT "datasets/Ranging_Simulation/M3500_heavy-tailed"
#define DATA_M_MM "datasets/Ranging_Simulation/M3500_multimodal"
#define DATA_M_S "datasets/Ranging_Simulation/M3500_skewed"
#define DATA_GT_M "datasets/Ranging_Simulation/M3500"

bool RunRanging(const std::string& DataSet,
                const std::string& GroundTruth,
               libRSF::StateDataSet &Result,
               libRSF::SensorDataSet &GT)
{
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(GroundTruth) + std::string("_GT.txt"), GT);

  /** assign all arguments to string vector*/
  std::vector<std::string> Arguments;
  Arguments.emplace_back(CONFIG);
  Arguments.emplace_back(std::string(DataSet) + std::string("_Input.txt"));
  Arguments.emplace_back("Result_App_GNSS.txt"); // shouldn't be necessary, not writing

  /** parse command line arguments */
  libRSF::FactorGraphConfig Config;
  Config.ReadCommandLineOptions(Arguments);

  /** disable cov estimation for tests to save time */
  Config.Solution.EstimateCov = false;

  /** run optimization */
  return CreateGraphAndSolve(Config, Result) > 0;
}

TEST(App_Ranging_2D, M3500_heavy_tailed)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunRanging(DATA_M_HT, DATA_GT_M, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 0.23, 0.01);
}

TEST(App_Ranging_2D, M3500_multimodal)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunRanging(DATA_M_MM, DATA_GT_M, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 0.24, 0.01);
}

TEST(App_Ranging_2D, M3500_skewed)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunRanging(DATA_M_S, DATA_GT_M, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 0.23, 0.01);
}