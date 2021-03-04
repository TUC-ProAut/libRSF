/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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

TEST(IV19_GNSS, smartLoc_Berlin_Potsdamer_Platz_gauss)
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
    double ate = libRSF::ATE(libRSF::SensorType::Point3, Gt, POSITION_STATE, Result);

    std::cout << "ATE: " << ate << std::endl;

    EXPECT_LT(ate,100.0);
}

TEST(IV19_GNSS, smartLoc_Berlin_Potsdamer_Platz_stsm_vbi)
{
    /** assign all arguments to string vector*/
    std::vector<std::string> Arguments;
    Arguments.push_back("datasets/smartLoc/Berlin_Potsdamer_Platz_Input.txt");
    Arguments.push_back("Result_smartLoc_Berlin_Potsdamer_Platz_gauss.txt"); // shouldn't be neccesary, not writing
    Arguments.push_back("error:");
    Arguments.push_back("stsm_vbi");

    /** calculate example */
    libRSF::StateDataSet Result;
    std::string OutputFile;

    ASSERT_FALSE(CreateGraphAndSolve(Arguments, Result, OutputFile)) << "Error calculating example";

    /** read gt */
    libRSF::SensorDataSet Gt;
    libRSF::ReadDataFromFile("datasets/smartLoc/Berlin_Potsdamer_Platz_GT.txt", Gt);

    /** calculate RMSE */
    double ate = libRSF::ATE(libRSF::SensorType::Point3, Gt, POSITION_STATE, Result);

    std::cout << "ATE: " << ate << std::endl;

    EXPECT_LT(ate,30.0);
}