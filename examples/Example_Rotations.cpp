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
* @file Example_Rotation.cpp
* @author Tim Pfeifer
* @date 08.05.2020
* @brief An example application to test different rotational functions.
* @copyright GNU Public License.
*
*/

#include "libRSF.h"

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** create rotation */
  double DeltaTime = 0.01;
  libRSF::Vector3 TurnRate;
  TurnRate << 0.0, M_PI*sqrt(2.0), M_PI*sqrt(2.0);

  /**convert to quaternion */
  libRSF::Quaternion Quat1 = libRSF::AngularVelocityToQuaternion<double>(TurnRate, DeltaTime);
  libRSF::Quaternion Quat2 = libRSF::RPYToQuaternion<double>(TurnRate(0)*DeltaTime, TurnRate(1)*DeltaTime, TurnRate(2)*DeltaTime);

  /** repeat rotation */
  libRSF::Quaternion QuatAA(1,0,0,0);
  libRSF::Quaternion QuatRPY(1,0,0,0);

  for (int n = 0; n < 1.0/DeltaTime; n++)
  {
    QuatAA = QuatAA*Quat1;
    QuatRPY = QuatRPY*Quat2;
  }

  /** print resulting quaternions */
  PRINT_LOGGING("Angle Axis conversion: ", QuatAA.vec().transpose());
  PRINT_LOGGING("Euler conversion:      ", QuatRPY.vec().transpose());

  /** convert back to angular velocities */
  libRSF::Vector3 TurnRateEuler1 = libRSF::QuaternionToRPYEigen<double>(Quat1)/DeltaTime;
  libRSF::Vector3 TurnRateEuler2 = libRSF::QuaternionToRPY<double>(Quat1)/DeltaTime;
  libRSF::Vector3 TurnRateAngleAxis = libRSF::QuaternionToAngularVelocity<double>(Quat1, DeltaTime);

  std::cout << std::endl;

  /** print angular velocities */
  PRINT_LOGGING("True angular velocities:                   ", TurnRate.transpose());
  PRINT_LOGGING("Converted angular velocities Euler Eigen:  ", TurnRateEuler1.transpose());
  PRINT_LOGGING("Converted angular velocities Euler libRSF: ", TurnRateEuler2.transpose());
  PRINT_LOGGING("Converted angular velocities Angle Axis:   ", TurnRateAngleAxis.transpose());


  return 0;
}
