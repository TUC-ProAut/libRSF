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
 * @file Test_App_SLAM.cpp
 * @author Tim Pfeifer
 * @date 31 Jan 2023
 * @brief File containing a test for an application that estimates a 2D pose based on odometry and loop-closures.
 * @copyright GNU Public License.
 *
 */

#include "../applications/App_SLAM.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

/** define paths */
#define CONFIG "config/Default_SLAM_2D.yaml"
#define DATA_LH "datasets/SLAM/Lecture_Hall"
#define DATA_C "datasets/SLAM/Corridor"

bool RunSLAM(const std::string& DataSet,
             libRSF::StateDataSet &Result,
             libRSF::SensorDataSet &GT)
{
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(DataSet) + std::string("_GT.txt"), GT);

  /** assign all arguments to string vector*/
  std::vector<std::string> Arguments;
  Arguments.emplace_back(CONFIG);
  Arguments.emplace_back(std::string(DataSet) + std::string("_Input.txt"));
  Arguments.emplace_back("Result_App_SLAM.txt"); // shouldn't be necessary, not writing

  /** parse command line arguments */
  libRSF::FactorGraphConfig Config;
  Config.ReadCommandLineOptions(Arguments);

  /** disable cov estimation for tests to save time */
  Config.Solution.EstimateCov = false;

  /** run optimization */
  if (CreateGraphAndSolve(Config, Result) > 0)
  {
    return false;
  }

  /** align trajectory and GT*/
  libRSF::StateDataSet ResultAligned;
  libRSF::AlignTrajectory2D(GT, POSITION_STATE, ORIENTATION_STATE, Result, ResultAligned);
  Result = ResultAligned;

  return true;
}

TEST(App_SLAM, Corridor)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_TRUE(RunSLAM(DATA_C, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 1.18, 0.1); // tolerance is high because of non-synchronized GT
}

TEST(App_SLAM, Lecture_Hall)
{
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_TRUE(RunSLAM(DATA_LH, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  EXPECT_NEAR(ATE, 0.49, 0.1); // tolerance is high because of non-synchronized GT
}