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
* @file Example_Odometry.cpp
* @author Tim Pfeifer
* @date 19.05.2020
* @brief An example application to compare different odometry implementations.
* @copyright GNU Public License.
*
*/

#include "libRSF.h"

#define COV_VEL 0.1
#define COV_TR 0.1

#define STEPS 5

int main(int ArgC, char** ArgV)
{
  (void)ArgC;
  google::InitGoogleLogging(*ArgV);

  /** create odometry data */
  libRSF::Vector3 Velocity;
  Velocity.x() = 1.0;
  Velocity.y() = 0.1;
  Velocity.z() = 0.1;

  libRSF::Vector3 TurnRate;
  TurnRate.x() = 0.001;
  TurnRate.y() = 0.001;
  TurnRate.z() = M_PI*2.0*0.1;

  std::vector<libRSF::Data> OdomMeasurements;
  for (int t = 0; t < STEPS; t++)
  {
    const double Time = (t + 1)*0.1;

    /** fixed movement */
    libRSF::Data Odom(libRSF::DataType::Odom3, Time);
    Odom.setMean((libRSF::Vector6() << Velocity, TurnRate).finished());
    Odom.setCovarianceDiagonal((libRSF::Vector6() << COV_VEL, COV_VEL, COV_VEL, COV_TR, COV_TR, COV_TR).finished());
    OdomMeasurements.emplace_back(Odom);
  }

  /** create factor graph */
  libRSF::FactorGraph Graph;
  Graph.addState("Position_Odom", libRSF::DataType::Point3, 0.0);
  Graph.addState("Position_Odom_Int", libRSF::DataType::Point3, 0.0);

  Graph.addState("Quaternion_Odom", libRSF::DataType::Quaternion, 0.0);
  Graph.addState("Quaternion_Odom_Int", libRSF::DataType::Quaternion, 0.0);

  /** freeze first state */
  Graph.setConstant("Position_Odom", 0.0);
  Graph.setConstant("Position_Odom_Int", 0.0);

  Graph.setConstant("Quaternion_Odom", 0.0);
  Graph.setConstant("Quaternion_Odom_Int", 0.0);

  /** normal (non-integrating) factor */
  double LastTimestamp = 0.0;
  for (const libRSF::Data &Odom : OdomMeasurements)
  {
    const double TimeStamp = Odom.getTimestamp();

    /** add state */
    Graph.addState("Position_Odom", libRSF::DataType::Point3, TimeStamp);
    Graph.addState("Quaternion_Odom", libRSF::DataType::Quaternion, TimeStamp);

    /** error model */
    libRSF::GaussianDiagonal<6> Noise;
    Noise.setCovarianceDiagonal(Odom.getCovarianceDiagonal());

    /** add factor */
    Graph.addFactor<libRSF::FactorType::Odom6>(libRSF::StateID("Position_Odom", LastTimestamp),
                                               libRSF::StateID("Quaternion_Odom", LastTimestamp),
                                               libRSF::StateID("Position_Odom", TimeStamp),
                                               libRSF::StateID("Quaternion_Odom", TimeStamp),
                                               Odom, Noise);
    LastTimestamp = TimeStamp;
  }

  /** integration and pose factor */
  libRSF::OdometryIntegrator OdomInt;
  LastTimestamp = 0.0;
  for (const libRSF::Data &Odom : OdomMeasurements)
  {
    const double TimeStamp = Odom.getTimestamp();

    /** integrate */
    OdomInt.addMeasurement(Odom, TimeStamp-LastTimestamp);

    /** get integrated pose */
    libRSF::Data PoseInt(libRSF::DataType::Pose3, TimeStamp);
    PoseInt.setMean(OdomInt.getJointPose());

    /** get integrated noise model */
    libRSF::GaussianFull<6> NoiseInt;
    NoiseInt.setCovarianceMatrix(OdomInt.getJointCovOnManifold());

    /** add integrated factor (Normally, we would just integrate all and add one factor. This is for evaluation of the intermediate steps.) */
    Graph.addState("Position_Odom_Int", libRSF::DataType::Point3, TimeStamp);
    Graph.addState("Quaternion_Odom_Int", libRSF::DataType::Quaternion, TimeStamp);
    Graph.addFactor<libRSF::FactorType::BetweenPose3>(libRSF::StateID("Position_Odom_Int", 0.0),
                                                      libRSF::StateID("Quaternion_Odom_Int", 0.0),
                                                      libRSF::StateID("Position_Odom_Int", TimeStamp),
                                                      libRSF::StateID("Quaternion_Odom_Int", TimeStamp),
                                                      PoseInt, NoiseInt);

    LastTimestamp = TimeStamp;
  }

  /** optimize both graphs */
  Graph.solve();
  Graph.computeCovariance("Position_Odom");
  Graph.computeCovariance("Position_Odom_Int");

  Graph.computeCovariance("Quaternion_Odom");
  Graph.computeCovariance("Quaternion_Odom_Int");

  /** output result */
  for (const libRSF::Data &Odom : OdomMeasurements)
  {
    const double TimeStamp = Odom.getTimestamp();

    std::cout <<"########################### Timestamp: "<< TimeStamp << " ###########################" <<std::endl;

    std::cout << "Factor  Odom - Translation: " << Graph.getStateData().getElement("Position_Odom", TimeStamp).getMean().transpose()
                                        << " ### "
                                        << Graph.getStateData().getElement("Position_Odom", TimeStamp).getCovarianceDiagonal().transpose()
                                        << std::endl;

    std::cout << "Integr. Odom - Translation: " << Graph.getStateData().getElement("Position_Odom_Int", TimeStamp).getMean().transpose()
                                        << " ### "
                                        << Graph.getStateData().getElement("Position_Odom_Int", TimeStamp).getCovarianceDiagonal().transpose()
                                        << std::endl;

    std::cout << std::endl;

    std::cout << "Factor  Odom - Rotation:   " << Graph.getStateData().getElement("Quaternion_Odom", TimeStamp).getMean().transpose()
                                        << " ### "
                                        << Graph.getStateData().getElement("Quaternion_Odom", TimeStamp).getCovarianceDiagonal().transpose()
                                        << std::endl;

    std::cout << "Integr. Odom - Rotation:   " << Graph.getStateData().getElement("Quaternion_Odom_Int", TimeStamp).getMean().transpose()
                                        << " ### "
                                        << Graph.getStateData().getElement("Quaternion_Odom_Int", TimeStamp).getCovarianceDiagonal().transpose()
                                        << std::endl;

    std::cout << std::endl;
  }

  return 0;
}
