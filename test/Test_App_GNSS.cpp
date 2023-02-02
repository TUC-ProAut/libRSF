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
 * @file Test_App_GNSS.cpp
 * @author Tim Pfeifer
 * @date 30 Jan 2023
 * @brief File containing a test for an application that estimates a 3D pose based on pseudo-range measurements.
 * @copyright GNU Public License.
 *
 */

#include "../applications/App_GNSS.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

/** define paths */
#define CONFIG "config/Default_GNSS.yaml"
#define DATA_BPB "datasets/smartLoc-RTKLIB/Berlin_Potsdamer_Platz_RTK"
#define DATA_BGM "datasets/smartLoc-RTKLIB/Berlin_Gendarmenmarkt_RTK"
#define DATA_FMT "datasets/smartLoc-RTKLIB/Frankfurt_Main_Tower_RTK"
#define DATA_FWT "datasets/smartLoc-RTKLIB/Frankfurt_Westend_Tower_RTK"

bool RunGNSS(const std::string& DataSet,
             libRSF::StateDataSet &Result,
             libRSF::SensorDataSet &GT)
{
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(DataSet) + std::string("_GT.txt"), GT);

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

TEST(App_GNSS, Berlin_Potsdamer_Platz)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_BPB, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 7.9, 0.1);
}

TEST(App_GNSS, Frankfurt_Main_Tower)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_FMT, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 13.9, 0.1);
}