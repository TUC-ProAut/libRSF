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

TEST(App_Robust_Models_2D, MaxSumMix)
{
  /** configure arguments */
  const double mean = 0;
  const std::string meanStr = std::to_string(mean);
  const int nPointsPerDim = 10;
  std::vector<std::string> Arguments{
  "empty", "empty", "Data_1D_Output.txt", std::to_string(nPointsPerDim), "8", "MaxSumMix",
  meanStr, meanStr, meanStr, meanStr, "0.5", "0", "0", "1", "2", "0", "0", "5", "0.35", "0.65"};
  libRSF::FactorGraphConfig Config;
  Config.ReadYAMLOptions(Arguments.at(0));

  /** datasets that store results*/
  libRSF::StateDataSet CostSurfaceData;
  libRSF::StateDataSet PreOptimizationData;
  libRSF::StateDataSet PostOptimizationData;
  libRSF::StateDataSet SolverData;

  ASSERT_FALSE(CreateGraphAndSolve(Arguments,Config,CostSurfaceData,PreOptimizationData,PostOptimizationData,SolverData)) << "Error calculating example";

  /*
  libRSF::SensorDataSet Expected;
  for (int kPose = 0; kPose<nPointsPerDim*nPointsPerDim; kPose++)
  {
      Expected.addElement(libRSF::Data("point2 1.0 " + mean + " " + mean + " 0.0 0.0 0.0 0.0"));
  }

  double maxAbsErrorMean = libRSF::MaxAbsError(libRSF::DataType::Point2,
                                                     Expected,
                                                     POSITION_STATE,
                                                     PostOptimizationData,
                                                     libRSF::DataElement::Mean);
  std::cout << "MaxAbsErrorMean:" << maxAbsErrorMean << std::endl;

  EXPECT_LT(maxAbsErrorMean,1e-3);
  */

  libRSF::Vector2 MeanVect;
  MeanVect << mean, mean;

  double Time;
  // iterate Result

    /** initialize maximum error */
    double maxAbsError = 0;

    /** get first timestamp */
    PostOptimizationData.getTimeFirst(POSITION_STATE, Time);

    do
    {
      int NumberOfStates = PostOptimizationData.countElement(POSITION_STATE,Time);
      /** get data at this timestamp */
      for (int nState = 0; nState < NumberOfStates; ++nState)
      {
        libRSF::Data DataResult;
        PostOptimizationData.getElement(POSITION_STATE, Time, nState, DataResult);

        /** get maximum difference */
        const double maxAbsErrorTmp =  (DataResult.getValue(libRSF::DataElement::Mean)-MeanVect).cwiseAbs().maxCoeff();

        /** store maximum of loop */
        if(maxAbsErrorTmp > maxAbsError)
        {
          maxAbsError = maxAbsErrorTmp;
        }
      }
    }
    while(PostOptimizationData.getTimeNext(POSITION_STATE, Time, Time));

  std::cout << "MaxAbsError:" << maxAbsError << std::endl;

  EXPECT_LT(maxAbsError,0.01);
}

// main provided by linking to gtest_main