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
 * @file Test_IV19_GNSS.cpp
 * @author Tim Pfeifer and Leopold Mauersberger
 * @date 04 Mar 2021
 * @brief File containing a test for an application for 3D pose estimation based on pseudo range measurements. This special version is made for the Intelligent Vehicles Symposium 2019.
 * @copyright GNU Public License.
 *
 */

#include "../applications/IV19_GNSS.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

/** define paths */
#define DATA_BPB "datasets/smartLoc/Berlin_Potsdamer_Platz"
#define DATA_BGM "datasets/smartLoc/Berlin_Gendarmenmarkt"
#define DATA_FMT "datasets/smartLoc/Frankfurt_Main_Tower"
#define DATA_FWT "datasets/smartLoc/Frankfurt_Westend_Tower"

bool RunGNSS(const std::string& DataSet,
             const std::string& ErrorModel,
             libRSF::StateDataSet &Result,
             libRSF::SensorDataSet &GT)
{
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(DataSet) + std::string("_GT.txt"), GT);

  /** assign all arguments to string vector*/
  std::vector<std::string> Arguments;
  Arguments.push_back(std::string(DataSet) + std::string("_Input.txt"));
  Arguments.emplace_back("Result.txt"); // shouldn't be necessary, not writing
  Arguments.emplace_back("error:");
  Arguments.push_back(ErrorModel);

  /** dummy argument */
  std::string OutputFile;

  /** run optimization */
  return CreateGraphAndSolve(Arguments, Result, OutputFile) > 0;
}

TEST(IV19_GNSS, Berlin_Potsdamer_Platz_Gaussian)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_BPB, "gauss", Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 68, 1.0);
}

TEST(IV19_GNSS, Berlin_Potsdamer_Platz_VBI_SM)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_BPB, "stsm_vbi", Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 19, 1.0);
}

TEST(IV19_GNSS, Berlin_Potsdamer_Platz_VBI_MM)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_BPB, "stmm_vbi", Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 19, 1.0);
}

TEST(IV19_GNSS, Berlin_Gendarmenmarkt_Gaussian)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_BGM, "gauss", Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 51, 1.0);
}

TEST(IV19_GNSS, Frankfurt_Main_Tower_EM_MM)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_FMT, "stmm", Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 40, 1.0);
}

TEST(IV19_GNSS, Frankfurt_Westend_Tower_EM_SM)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunGNSS(DATA_FWT, "stsm", Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 42, 1.0);
}
