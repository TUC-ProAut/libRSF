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
 * @file IMUFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Simple IMU integration factor.
 * @copyright GNU Public License.
 *
 */

#ifndef IMUFACTOR_H
#define IMUFACTOR_H

#include "BaseFactor.h"
#include "../geometric_models/IMUModel.h"

namespace libRSF
{
  template <typename ErrorType>
  class IMUFactor : public BaseFactor<ErrorType, true, true, 3, 4, 9, 3, 4, 9>
  {
    public:
      /** construct factor and store measurement */
      IMUFactor(ErrorType &Error, const Data &IMUMeasurement, double DeltaTime)
      {
        this->Error_ = Error;
        this->MeasurementVector_.resize(6);
        this->MeasurementVector_ = IMUMeasurement.getMean();
        this->DeltaTime_ = DeltaTime;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 15> Evaluate(const T* const PointOld, const T* const QuatOld, const T* const SpeedBiasOld,
                              const T* const PointNew, const T* const QuatNew, const T* const SpeedBiasNew,
                              const Vector3 &Acceleration, const Vector3 &TurnRate, const double DeltaTime) const
      {
        /** map pointer to vectors */
        MatrixRefConst<T, 3, 1> P1(PointOld);
        MatrixRefConst<T, 3, 1> P2(PointNew);

        MatrixRefConst<T, 3, 1> V1(SpeedBiasOld);
        MatrixRefConst<T, 3, 1> V2(SpeedBiasNew);

        MatrixRefConst<T, 3, 1> BiasAcc1(SpeedBiasOld + 3);
        MatrixRefConst<T, 3, 1> BiasAcc2(SpeedBiasNew + 3);

        MatrixRefConst<T, 3, 1> BiasTR1(SpeedBiasOld + 6);
        MatrixRefConst<T, 3, 1> BiasTR2(SpeedBiasNew + 6);

        /** estimate measurements */
        VectorT<T, 3> VelocityEst, TurnRateEst, AccelerationEst;
        IMUModel<T>::applyBackward(PointOld, QuatOld, SpeedBiasOld, PointNew, QuatNew, SpeedBiasNew, VelocityEst, AccelerationEst, TurnRateEst, DeltaTime);

        /** error = estimated measurement - measurement */
        VectorT<T, 15> Error;
        Error.template segment<3>(0) = AccelerationEst - Acceleration;
        Error.template segment<3>(3) = VelocityEst - V1;
        Error.template segment<3>(6) = NormalizeAngleVelocityVector<T, 3>(TurnRateEst - TurnRate.template cast<T>(), DeltaTime);

        /** simple constant model for bias error */
        Error.template segment<3>(9) = BiasAcc2 - BiasAcc1;
        Error.template segment<3>(12) = BiasTR2 - BiasTR1;

        return Error;
      }

      /** combine probabilistic and geometric model */
      template <typename T>
      bool operator()(const T* const PointOld, const T* const QuatOld, const T* const SpeedBiasOld,
                      const T* const PointNew, const T* const QuatNew, const T* const SpeedBiasNew,
                      T* Error) const
      {
        this->Error_.weight(this->Evaluate(PointOld, QuatOld, SpeedBiasOld,
                                           PointNew, QuatNew, SpeedBiasNew,
                                           this->MeasurementVector_.segment(0, 3),
                                           this->MeasurementVector_.segment(3, 3),
                                           this->DeltaTime_),
                            Error);
        return true;
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        IMUModel<double>::applyForward(StatePointers[0],
                                       StatePointers[1],
                                       StatePointers[2],
                                       StatePointers[3],
                                       StatePointers[4],
                                       StatePointers[5],
                                       this->MeasurementVector_.head(3),
                                       this->MeasurementVector_.tail(3),
                                       this->DeltaTime_);
      }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::IMUSimple, ErrorType> {using Type = IMUFactor<ErrorType>;};
}

#endif // IMUFACTOR_H
