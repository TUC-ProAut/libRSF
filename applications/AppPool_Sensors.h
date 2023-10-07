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
 * @file AppPool_Sensors.h
 * @author Tim Pfeifer
 * @date 09 August 2019
 * @brief Contains a set of functions, related to specific sensors.
 * @copyright GNU Public License.
 *
 */

#ifndef APPPOOL_SENSORS_H
#define APPPOOL_SENSORS_H

#include <cmath>

#include "AppPool_Adaptive.h"
#include "AppPool_Defines.h"
#include "libRSF.h"

void AddGNSS(libRSF::FactorGraph &Graph,
             const libRSF::FactorGraphConfig &Config,
             const libRSF::SensorDataSet &Measurements,
             double TimeOld,
             double TimeNow);

void AddUWB(libRSF::FactorGraph &Graph,
            const libRSF::FactorGraphConfig &Config,
            const libRSF::SensorDataSet &Measurements,
            double TimeOld,
            double TimeNow);

void AddRange3(libRSF::FactorGraph &Graph,
               const libRSF::FactorGraphConfig &Config,
               const libRSF::Data &Range,
               double TimePosition);

void AddRange2(libRSF::FactorGraph &Graph,
               const libRSF::FactorGraphConfig &Config,
               const libRSF::Data &Range,
               double TimePosition);

void AddRangeToPoint2(libRSF::FactorGraph &Graph,
                      const libRSF::FactorGraphConfig &Config,
                      const libRSF::Data &Range,
                      double TimePosition);

void AddMotionModel(libRSF::FactorGraph &Graph,
                    const libRSF::FactorGraphConfig &Config,
                    double TimeOld,
                    double TimeNow);

void AddOdometry(libRSF::FactorGraph &Graph,
                 libRSF::FactorType FactorConfig,
                 libRSF::DataType SensorConfig,
                 const libRSF::SensorDataSet &Measurements,
                 double TimeOld,
                 double TimeNow);

void AddRadar(libRSF::FactorGraph &Graph,
              const libRSF::FactorGraphConfig &Config,
              const libRSF::SensorDataSet &Measurements,
              double TimeOld,
              double TimeNow);

void AddLaser(libRSF::FactorGraph &Graph,
              const libRSF::FactorGraphConfig &Config,
              const libRSF::SensorDataSet &Measurements,
              double TimeOld,
              double TimeNow);

void AddClockModel(libRSF::FactorGraph &Graph,
                   const libRSF::FactorGraphConfig &Config,
                   const std::string &ClockErrorNameStart,
                   const std::string &ClockDriftNameStart,
                   const std::string &ClockErrorNameEnd,
                   const std::string &ClockDriftNameEnd,
                   double TimeNow);

void AddIMU(libRSF::FactorGraph &Graph,
            const libRSF::FactorGraphConfig &Config,
            const libRSF::SensorDataSet &Measurements,
            double TimeOld,
            double TimeNow);

void AddPressure(libRSF::FactorGraph &Graph,
                 libRSF::FactorType FactorConfig,
                 const libRSF::SensorDataSet &Measurements,
                 double TimeOld,
                 double TimeNow);

void AddPseudorange3(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     const libRSF::Data &Pseudorange,
                     double TimePosition);

void AddInterSystemBiasModel(libRSF::FactorGraph &Graph,
                             const libRSF::FactorGraphConfig &Config,
                             const std::string &ISBName,
                             double TimeBias);

std::string addSatSysToName(const std::string &BaseName, int SatSys);

bool AddLoopClosures(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     const libRSF::SensorDataSet &Measurements,
                     double TimeOld,
                     double TimeNow);

template <libRSF::FactorType FactorType, int Dim>
void AddLoopClosureGeneric(libRSF::FactorGraph &Graph,
                           const libRSF::FactorGraphConfig &Config,
                           const libRSF::Data &Loop)
{
  /** get standard deviation in [m] */
  libRSF::VectorStatic<Dim> StdDev;

  /** find closest timestamps */
  double Time1 = NAN, Time2 = NAN;
  Time1 = Loop.getTimestamp();
  Time2 = Loop.getValue(libRSF::DataElement::TimestampRef)(0);
  Graph.getStateData().getTimeCloseTo(POSITION_STATE, Time1, Time1);
  Graph.getStateData().getTimeCloseTo(POSITION_STATE, Time2, Time2);

  /** add rotations if required */
  libRSF::StateList ListOfStates;
  if (FactorType == libRSF::FactorType::LoopPose2)
  {
      /** add  full pose */
      ListOfStates.add(libRSF::StateID(POSITION_STATE, Time1));
      ListOfStates.add(libRSF::StateID(ORIENTATION_STATE, Time1));
      ListOfStates.add(libRSF::StateID(POSITION_STATE, Time2));
      ListOfStates.add(libRSF::StateID(ORIENTATION_STATE, Time2));

      StdDev = Config.LoopClosure.Parameter.tail(Dim);
  }
  else
  {
      /** add only position */
      ListOfStates.add(libRSF::StateID(POSITION_STATE, Time1));
      ListOfStates.add(libRSF::StateID(POSITION_STATE, Time2));

      StdDev = libRSF::VectorStatic<Dim>::Ones() * Config.LoopClosure.Parameter(1);
  }

  /** add factor with right error model */
  switch(Config.LoopClosure.ErrorModel.Type)
  {
    case libRSF::ErrorModelType::Gaussian:
    {
      libRSF::GaussianDiagonal<Dim> Noise;
      Noise.setStdDevDiagonal(StdDev);

      Graph.addFactor<FactorType>(ListOfStates, Noise, false);
    }
    break;
    case libRSF::ErrorModelType::GMM:
    {
      /** init model */
      const libRSF::GaussianMixture<Dim> GMM(Config.LoopClosure.ErrorModel.GMM);

      /** Max-Mixture GMM [Olson et al.]*/
      if(Config.LoopClosure.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
      {
        libRSF::MaxMix<Dim> MixtureNoise(GMM);
        Graph.addFactor<FactorType>(ListOfStates, MixtureNoise, false);
      }
      /** Sum-Mixture GMM [Rosen et al.]*/
      else if(Config.LoopClosure.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
      {
        libRSF::SumMix<Dim> MixtureNoise(GMM);
        Graph.addFactor<FactorType>(ListOfStates, MixtureNoise, false);
      }
      /** Max-Sum-Mix-Mixture GMM [Pfeifer et al.]*/
      else if(Config.LoopClosure.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxSumMix)
      {
        libRSF::MaxSumMix<Dim> MixtureNoise(GMM);
        Graph.addFactor<FactorType>(ListOfStates, MixtureNoise, false);
      }
      else
      {
        PRINT_ERROR("Wrong mixture type!");
      }
    }
    break;

    case libRSF::ErrorModelType::SC:
    {
      libRSF::GaussianDiagonal<Dim> Noise;
      Noise.setStdDevDiagonal(StdDev);

      /**add switch variable */
      Graph.addState(SWITCH_STATE, libRSF::DataType::Switch, Time2);
      ListOfStates.add(libRSF::StateID(SWITCH_STATE, Time2));

      /** create Switchable Constraints error model */
      libRSF::SwitchableConstraints<Dim, libRSF::GaussianDiagonal<Dim>> SC(Noise, Config.LoopClosure.ErrorModel.Parameter);
      Graph.addFactor<FactorType>(ListOfStates, SC);
    }
    break;

    default:
      PRINT_ERROR("Wrong error type: ", Config.LoopClosure.ErrorModel.Type);
      break;
  }
}

template <libRSF::FactorType RangeFactorType>
void AddRangeGeneric(libRSF::FactorGraph &Graph,
                     const libRSF::FactorGraphConfig &Config,
                     const libRSF::Data &Range,
                     const double TimePosition)
{
  double const TimeRange = Range.getTimestamp();

  /** construct states if missing and create state list */
  libRSF::StateList ListOfPoints;
  switch (RangeFactorType)
  {
    case libRSF::FactorType::Range2:
      Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point2, TimePosition);
      ListOfPoints.add(POSITION_STATE, TimePosition);
      break;

    case libRSF::FactorType::Range3:
      Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimePosition);
      ListOfPoints.add(POSITION_STATE, TimePosition);
      break;

    case libRSF::FactorType::RangeToPoint2:
    {
      /** ego position*/
      Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point2, TimePosition);
      ListOfPoints.add(POSITION_STATE, TimePosition);

      /** landmark position*/
      const std::string LM_ID_STATE = LANDMARK_STATE + std::to_string(Range.getValue(libRSF::DataElement::SatID)(0));
      if (!Graph.getStateData().checkElement(LM_ID_STATE, 0.0))
      {
        /** add landmark state */
        Graph.addStateWithCheck(LM_ID_STATE, libRSF::DataType::PointID2, 0.0);
        /** store ID of landmark */
        Graph.getStateData().getElement(LM_ID_STATE, 0.0).setValue(libRSF::DataElement::ID, Range.getValue(libRSF::DataElement::SatID));
        /** initialize landmark at current position */
        Graph.getStateData().getElement(LM_ID_STATE, 0.0).setMean(Graph.getStateData().getElement(POSITION_STATE, TimePosition).getMean());
      }

      ListOfPoints.add(LM_ID_STATE, 0.0);
      break;
    }

    case libRSF::FactorType::RangeToPoint3:
    {
      /** ego position*/
      Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimePosition);
      ListOfPoints.add(POSITION_STATE, TimePosition);

      /** landmark position*/
      const std::string LM_ID_STATE = LANDMARK_STATE + std::to_string(Range.getValue(libRSF::DataElement::SatID)(0));
      if (!Graph.getStateData().checkElement(LM_ID_STATE, 0.0))
      {
        /** add landmark state */
        Graph.addStateWithCheck(LM_ID_STATE, libRSF::DataType::Point3, 0.0);
        /** store ID of landmark */
        Graph.getStateData().getElement(LM_ID_STATE, 0.0).setValue(libRSF::DataElement::ID, Range.getValue(libRSF::DataElement::SatID));
        /** initialize landmark at current position */
        Graph.getStateData().getElement(LM_ID_STATE, 0.0).setMean(Graph.getStateData().getElement(POSITION_STATE, TimePosition).getMean());
      }
      ListOfPoints.add(LM_ID_STATE, 0.0);
      break;
    }

    default:
      PRINT_ERROR("Factor type not handled: ", RangeFactorType);
      break;

  }

  /** add pseudorange */
  switch (Config.Ranging.ErrorModel.Type)
  {
    case libRSF::ErrorModelType::Gaussian:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(Range.getStdDevDiagonal());

      Graph.addFactor<RangeFactorType>(
          ListOfPoints,
          Range,
          Noise);
    }
    break;

    case libRSF::ErrorModelType::GMM:
    {
      /** Use default init */
      libRSF::GaussianMixture<1> const GMM(Config.Ranging.ErrorModel.GMM);

      /**Max-Mixture GMM [Olson et al.]*/
      if (Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
      {
        libRSF::MaxMix1 MixtureNoise(GMM);

        Graph.addFactor<RangeFactorType>(
            ListOfPoints,
            Range,
            MixtureNoise);
      }
      /**Sum-Mixture GMM [Rosen et al.]*/
      else if (Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
      {
        libRSF::SumMix1 MixtureNoise(GMM);

        Graph.addFactor<RangeFactorType>(
            ListOfPoints,
            Range,
            MixtureNoise);
      }
      /**LSE-Mixture GMM [Pfeifer et al.]*/
      else if (Config.Ranging.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxSumMix)
      {
        libRSF::MaxSumMix1 MixtureNoise(GMM);

        Graph.addFactor<RangeFactorType>(
            ListOfPoints,
            Range,
            MixtureNoise);
      }
      else
      {
        PRINT_ERROR("Wrong mixture type!");
      }
    }
    break;

    case libRSF::ErrorModelType::SC:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(Range.getStdDevDiagonal());

      /**add switch variable */
      Graph.addState(SWITCH_STATE, libRSF::DataType::Switch, TimeRange);
      const int SwitchIndex = Graph.getStateData().countElement(SWITCH_STATE, TimeRange) - 1;

      /** create Switchable Constraints error model */
      libRSF::SwitchableConstraints<1, libRSF::GaussianDiagonal<1>> SC(Noise, Config.Ranging.ErrorModel.Parameter);

      /** add to graph */
      Graph.addFactor<RangeFactorType>(
          ListOfPoints,
          libRSF::StateID(SWITCH_STATE, TimeRange, SwitchIndex),
          Range,
          SC);
    }
    break;

    case libRSF::ErrorModelType::DCS:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(Range.getStdDevDiagonal());

      Graph.addFactor<RangeFactorType>(
          ListOfPoints,
          Range,
          Noise,
          new libRSF::DCSLoss(Config.Ranging.ErrorModel.Parameter));
    }
    break;

    case libRSF::ErrorModelType::DCE:
    {
      /**add covariance variable */
      Graph.addState(COV_STATE, libRSF::DataType::Covariance1, TimeRange);
      const int CovIndex = Graph.getStateData().countElement(COV_STATE, TimeRange) - 1;
      const libRSF::Vector1 Cov = Range.getCovarianceDiagonal();

      /** set initial value */
      Graph.getStateData().getElement(COV_STATE, TimeRange, CovIndex).setMean(Cov);

      /** set lower bound */
      Graph.setLowerBound(COV_STATE, TimeRange, CovIndex, Cov);

      /** create Dynamic Covariance Estimation error model */
      libRSF::DynamicCovarianceEstimation<1> DCE(Cov);

      /** add to graph */
      Graph.addFactor<RangeFactorType>(ListOfPoints,
                                       libRSF::StateID(COV_STATE, TimeRange, CovIndex),
                                       Range,
                                       DCE);
    }
    break;

    case libRSF::ErrorModelType::cDCE:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(libRSF::Matrix11::Ones());

      Graph.addFactor<RangeFactorType>(
          ListOfPoints,
          Range,
          Noise,
          new libRSF::cDCELoss(Range.getStdDevDiagonal()(0)));
    }
    break;

    default:
      PRINT_ERROR("Error model not handled: ", Config.Ranging.ErrorModel.Type);
      break;
  }
}

template <libRSF::FactorType PseudorangeFactorType>
void AddPseudorange3Generic(libRSF::FactorGraph &Graph,
                            const libRSF::FactorGraphConfig &Config,
                            const libRSF::Data &Pseudorange,
                            const double TimePosition)
{
  const double TimeGNSS = Pseudorange.getTimestamp();

  /** construct position if missing */
  Graph.addStateWithCheck(POSITION_STATE, libRSF::DataType::Point3, TimePosition);

  /** construct clock error if missing */
  Graph.addStateWithCheck(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimeGNSS);

  /** collect states */
  libRSF::StateList States;
  States.add(POSITION_STATE, TimePosition);
  States.add(CLOCK_ERROR_STATE, TimeGNSS);

  /** add inter system bias */
  if constexpr (PseudorangeFactorType == libRSF::FactorType::Pseudorange3_Bias)
  {
    /** get string*/
    const std::string InterSystemBiasName = addSatSysToName(SYSTEM_BIAS_STATE, static_cast<int>(Pseudorange.getValue(libRSF::DataElement::SatSys)(0)));

    /** find closest ISB variable*/
    double TimeISB = NAN;
    Graph.getStateData().getTimeCloseTo(InterSystemBiasName, TimeGNSS, TimeISB);
    States.add(InterSystemBiasName, TimeISB);
  }

  /** add pseudorange */
  switch (Config.GNSS.ErrorModel.Type)
  {
    case libRSF::ErrorModelType::Gaussian:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());

      Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, Noise);
    }
    break;

    case libRSF::ErrorModelType::GMM:
    {
      /** Use default init */
      libRSF::GaussianMixture<1> const GMM(Config.GNSS.ErrorModel.GMM);

      /**Max-Mixture GMM [Olson et al.]*/
      if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
      {
        libRSF::MaxMix1 MixtureNoise(GMM);

        Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, MixtureNoise);
      }
      /**Sum-Mixture GMM [Rosen et al.]*/
      else if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
      {
        libRSF::SumMix1 MixtureNoise(GMM);

        Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, MixtureNoise);
      }
      /**MaxSumMix-Mixture GMM [Pfeifer et al.]*/
      else if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxSumMix)
      {
        libRSF::MaxSumMix1 MixtureNoise(GMM);

        Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, MixtureNoise);
      }
      else
      {
        PRINT_ERROR("Wrong mixture type!");
      }
    }
    break;

    case libRSF::ErrorModelType::SC:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());

      /**add switch variable */
      Graph.addState(SWITCH_STATE, libRSF::DataType::Switch, TimeGNSS);
      const int SwitchIndex = Graph.getStateData().countElement(SWITCH_STATE, TimeGNSS) - 1;
      States.add(SWITCH_STATE, TimeGNSS, SwitchIndex);

      /** create Switchable Constraints error model */
      libRSF::SwitchableConstraints<1, libRSF::GaussianDiagonal<1>> SC(Noise, Config.GNSS.ErrorModel.Parameter);

      /** add to graph */
      Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, SC);
    }
    break;

    case libRSF::ErrorModelType::DCS:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());

      Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, Noise,
                                             new libRSF::DCSLoss(Config.GNSS.ErrorModel.Parameter));
    }
    break;

    case libRSF::ErrorModelType::DCE:
    {
      /**add covariance variable */
      Graph.addState(COV_STATE, libRSF::DataType::Covariance1, TimeGNSS);
      const int CovIndex = Graph.getStateData().countElement(COV_STATE, TimeGNSS) - 1;
      const libRSF::Vector1 Cov = Pseudorange.getCovarianceDiagonal();
      States.add(COV_STATE, TimeGNSS, CovIndex);

      /** set initial value */
      Graph.getStateData().getElement(COV_STATE, TimeGNSS, CovIndex).setMean(Cov);

      /** set lower bound */
      Graph.setLowerBound(COV_STATE, TimeGNSS, CovIndex, Cov);
      //Graph.setUpperBound(COV_STATE, TimeGNSS, CovIndex, Cov*1e12); /** for better numerical stability*/

      /** create Dynamic Covariance Estimation error model */
      libRSF::DynamicCovarianceEstimation<1> DCE(Cov);

      /** add to graph */
      Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, DCE);
    }
    break;

    case libRSF::ErrorModelType::cDCE:
    {
      libRSF::GaussianDiagonal<1> Noise;
      Noise.setStdDevDiagonal(libRSF::Matrix11::Ones());

      Graph.addFactor<PseudorangeFactorType>(States, Pseudorange, Noise,
                                             new libRSF::cDCELoss(Pseudorange.getStdDevDiagonal()(0)));
    }
    break;

    default:
      PRINT_ERROR("Error model not handled: ", Config.Ranging.ErrorModel.Type);
      break;
  }
}

#endif  // APPPOOL_SENSORS_H
