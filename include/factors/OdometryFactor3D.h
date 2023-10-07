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
 * @file OdometryFactor3D.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Factors that connect two 3D poses with an odometry measurement. Different degrees of freedom are possible.
 * @copyright GNU Public License.
 *
 */

#ifndef ODOMETRYFACTOR3D_H
#define ODOMETRYFACTOR3D_H

#include "BaseFactor.h"
#include "../Geometry.h"
#include "../geometric_models/OdometryModel.h"

namespace libRSF
{
  template <typename ErrorType>
  class OdometryFactor3D4DOF_ECEF : public BaseFactor<ErrorType, true, true, 3, 1, 3, 1>
  {
    public:
      /** construct factor and store measurement */
      OdometryFactor3D4DOF_ECEF(ErrorType &Error, const Data &OdometryMeasurement, double DeltaTime)
      {
        this->Error_ = Error;

        this->MeasurementVector_.resize(6);
        this->MeasurementVector_ = OdometryMeasurement.getMean();
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 4> Evaluate(const T* const Pos1, const T* const Yaw1,
                             const T* const Pos2, const T* const Yaw2) const
      {
        const Vector6 Odometry = this->MeasurementVector_;

        /** estimate measurements */
        VectorT<T, 3> VelocityEst, TurnRateEst;
        OdometryModel4DOFECEF<T>::applyBackward(Pos1, Yaw1, Pos2, Yaw2, VelocityEst, TurnRateEst, this->DeltaTime_);

        /** error = estimated measurement - measurement */
        VectorT<T, 4> Error;
        Error.template head<3>() = VelocityEst - Odometry.template head<3>().template cast<T>();
        Error(3) = NormalizeAngleVelocity<T>(TurnRateEst(2) - Odometry(5), this->DeltaTime_);

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Yaw1,
                      const T* const Pos2, const T* const Yaw2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Yaw1,
                                                              Pos2, Yaw2),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        OdometryModel4DOFECEF<double>::applyForward(StatePointers[0],
                                                    StatePointers[1],
                                                    StatePointers[2],
                                                    StatePointers[3],
                                                    this->MeasurementVector_.head(3),
                                                    this->MeasurementVector_.tail(3),
                                                    this->DeltaTime_);
      }
  };

  template <typename ErrorType>
  class OdometryFactor3D4DOF : public BaseFactor<ErrorType, true, true, 3, 1, 3, 1>
  {
    public:
      /** construct factor and store measurement */
      OdometryFactor3D4DOF(ErrorType &Error, const Data &OdometryMeasurement, double DeltaTime)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(6);
        this->MeasurementVector_ = OdometryMeasurement.getMean();
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 4> Evaluate(const T* const Pos1, const T* const Yaw1,
                             const T* const Pos2, const T* const Yaw2) const
      {
        const Vector6 Odometry = this->MeasurementVector_;

        /** estimate measurements */
        VectorT<T, 3> VelocityEst, TurnRateEst;
        OdometryModel4DOF<T>::applyBackward(Pos1, Yaw1, Pos2, Yaw2, VelocityEst, TurnRateEst, this->DeltaTime_);

        /** error = estimated measurement - measurement */
        VectorT<T, 4> Error;
        Error.template head<3>() = VelocityEst - Odometry.template head<3>().template cast<T>();
        Error(3) = NormalizeAngleVelocity<T>(TurnRateEst(2) - Odometry(5), this->DeltaTime_);

        return Error;
      }

      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Yaw1,
                      const T* const Pos2, const T* const Yaw2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Yaw1,
                                                              Pos2, Yaw2),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        OdometryModel4DOF<double>::applyForward(StatePointers[0],
                                                StatePointers[1],
                                                StatePointers[2],
                                                StatePointers[3],
                                                this->MeasurementVector_.head(3),
                                                this->MeasurementVector_.tail(3),
                                                this->DeltaTime_);
      }
  };

  template <typename ErrorType>
  class OdometryFactor3D6DOF : public BaseFactor< ErrorType, true, true, 3, 4, 3, 4>
  {
    public:
      /** construct factor and store measurement */
      OdometryFactor3D6DOF(ErrorType &Error, const Data &OdometryMeasurement, double DeltaTime)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(6);
        this->MeasurementVector_ = OdometryMeasurement.getMean();
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 6> Evaluate(const T* const Pos1, const T* const Quat1,
                             const T* const Pos2, const T* const Quat2) const
      {
        const Vector6 Odometry = this->MeasurementVector_;

        /** estimate measurements */
        VectorT<T, 3> VelocityEst, TurnRateEst;
        OdometryModel6DOF<T>::applyBackward(Pos1, Quat1, Pos2, Quat2, VelocityEst, TurnRateEst, this->DeltaTime_);

        /** error = estimated measurement - measurement */
        VectorT<T, 6> Error;
        Error.template head<3>() = VelocityEst - Odometry.template head<3>().template cast<T>();
        Error.template tail<3>() = NormalizeAngleVelocityVector<T, 3>(TurnRateEst - Odometry.template tail<3>().template cast<T>(), this->DeltaTime_);

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T, typename... ParamsType>
      bool operator()(const T* const Pos1, const T* const Quat1,
                      const T* const Pos2, const T* const Quat2,
                      ParamsType... Params) const
      {
        return this->Error_.template weight<T>(this->Evaluate(Pos1, Quat1,
                                                              Pos2, Quat2),
                                               Params...);
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        OdometryModel6DOF<double>::applyForward(StatePointers[0],
                                                StatePointers[1],
                                                StatePointers[2],
                                                StatePointers[3],
                                                this->MeasurementVector_.head(3),
                                                this->MeasurementVector_.tail(3),
                                                this->DeltaTime_);
      }
  };

  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Odom4, ErrorType> {using Type = OdometryFactor3D4DOF<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Odom4_ECEF, ErrorType> {using Type = OdometryFactor3D4DOF_ECEF<ErrorType>;};
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::Odom6, ErrorType> {using Type = OdometryFactor3D6DOF<ErrorType>;};
}


#endif // ODOMETRYFACTOR3D_H
