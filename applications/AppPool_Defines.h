/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option); any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de);
 ***************************************************************************/

/**
 * @file AppPool_Defines.h
 * @author Tim Pfeifer
 * @date 09 August 2019
 * @brief Contains a set of defines for state identifiers.
 * @copyright GNU Public License.
 *
 */

#ifndef APPPOOL_DEFINES_H
#define APPPOOL_DEFINES_H

/** pose of the system */
#define POSE_STATE "Pose"
#define POSITION_STATE "Position"
#define LANDMARK_STATE "Landmark"
#define ORIENTATION_STATE "Orientation"
#define ANGLE_STATE "Angle"

/** helper variables for non-integrating IMU factor */
#define ORIENTATION_IMU_STATE "OrientationIMU"
#define POSITION_IMU_STATE "PositionIMU"

/** IMU bias and system speed */
#define IMU_STATE "IMUSpeedBias"

/** GNSS receiver clock */
#define CLOCK_ERROR_STATE "ClockError"
#define CLOCK_DRIFT_STATE "ClockDrift"

/** inter system bias */
#define SYSTEM_BIAS_STATE "SystemBias"

/** run time */
#define SOLVE_TIME_STATE "SolveTime"

/** for switchable constraints */
#define SWITCH_STATE "Switch"

/** for dynamic covariance estimation */
#define COV_STATE "Covariance"

/** error state for debugging */
#define ERROR_STATE "Error"

#endif // APPPOOL_DEFINES_H
