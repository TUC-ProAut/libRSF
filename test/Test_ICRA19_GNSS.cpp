/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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
 * @file Test_ICRA19_GNSS.cpp
 * @author Leopold Mauersberger
 * @date 18 Feb 2021
 * @brief Comparing the ICRA19_GNSS application results against sample solution
 * @copyright GNU Public License.
 *
 */

#include "../applications/ICRA19_GNSS.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

TEST(ICRA19_GNSS, smartLoc_Berlin_Potsdamer_Platz_gauss)
{
    /** assign all arguments to string vector*/
    std::vector<std::string> Arguments;
    Arguments.push_back("datasets/smartLoc/Berlin_Potsdamer_Platz_Input.txt");
    Arguments.push_back("Result_smartLoc_Berlin_Potsdamer_Platz_gauss.txt"); // shouldn't be neccesary, not writing
    Arguments.push_back("error:");
    Arguments.push_back("gauss");

    /** calculate example */
    libRSF::StateDataSet Result;
    std::string OutputFile;

    ASSERT_FALSE(CreateGraphAndSolve(Arguments, Result, OutputFile)) << "Error calculating example";

    /** read gt */
    libRSF::SensorDataSet Gt;
    libRSF::ReadDataFromFile("datasets/smartLoc/Berlin_Potsdamer_Platz_GT.txt", Gt);

    /** calculate RMSE */
    double ATE = libRSF::ATE(libRSF::DataType::Point3, Gt, POSITION_STATE, Result);

    std::cout << "ATE: " << ATE << std::endl;

    EXPECT_LT(ATE, 70.0);
}

TEST(ICRA19_GNSS, smartLoc_Berlin_Potsdamer_Platz_stsm)
{
    /** assign all arguments to string vector*/
    std::vector<std::string> Arguments;
    Arguments.push_back("datasets/smartLoc/Berlin_Potsdamer_Platz_Input.txt");
    Arguments.push_back("Result_smartLoc_Berlin_Potsdamer_Platz_gauss.txt"); // shouldn't be neccesary, not writing
    Arguments.push_back("error:");
    Arguments.push_back("stsm");

    /** calculate example */
    libRSF::StateDataSet Result;
    std::string OutputFile;

    ASSERT_FALSE(CreateGraphAndSolve(Arguments, Result, OutputFile)) << "Error calculating example";

    /** read gt */
    libRSF::SensorDataSet Gt;
    libRSF::ReadDataFromFile("datasets/smartLoc/Berlin_Potsdamer_Platz_GT.txt", Gt);

    /** calculate RMSE */
    double ATE = libRSF::ATE(libRSF::DataType::Point3, Gt, POSITION_STATE, Result);

    std::cout << "ATE: " << ATE << std::endl;

    EXPECT_LT(ATE,23.0);
}

/** main provided by linking to gtest_main */


