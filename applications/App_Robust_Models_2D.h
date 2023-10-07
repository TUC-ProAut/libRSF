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
* @file App_Robust_Models_2D.cpp
* @author Tim Pfeifer
* @date 18.03.2021
* @brief Header for a simple application to evaluate the robust error functions in 2D and the corresponding test.
* @copyright GNU Public License.
*
*/

#ifndef APP_ROBUST_MODELS_2D_H_INCLUDED
#define APP_ROBUST_MODELS_2D_H_INCLUDED

#include <string>
#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position"
#define SOLVE_TIME_STATE "SolveTime"

int CreateGraphAndSolve(std::vector<std::string> &Arguments,
                       libRSF::StateDataSet &CostSurfaceData,
                       libRSF::StateDataSet &PreOptimizationData,
                       libRSF::StateDataSet &PostOptimizationData,
                       libRSF::StateDataSet &SolverData);

#endif // APP_ROBUST_MODELS_2D_H_INCLUDED