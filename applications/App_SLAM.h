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
 * @file App_SLAM.h
 * @author Tim Pfeifer
 * @date 02 Sep 2019
 * @brief Simple 2D SLAM with robust factor graphs.
 * @copyright GNU Public License.
 *
 */

#ifndef APP_SLAM_H
#define APP_SLAM_H

#include "AppPool_Adaptive.h"
#include "AppPool_Defines.h"
#include "AppPool_Init.h"
#include "AppPool_Sensors.h"
#include "AppPool_Utility.h"

#include "libRSF.h"

int CreateGraphAndSolve(const libRSF::FactorGraphConfig &Config,
                        libRSF::StateDataSet &Result);

#endif // APP_SLAM_H
