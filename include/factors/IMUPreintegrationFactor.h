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
 * @file IMUPReintegrationFactor.h
 * @author Tim Pfeifer
 * @date 20.08.2020
 * @brief IMU pre-integration factor.
 * @copyright GNU Public License.
 *
 */

#ifndef IMUPREINTEGRATIONFACTOR_H
#define IMUPREINTEGRATIONFACTOR_H

#include "BaseFactor.h"
#include "../error_models/Gaussian.h"
#include "../geometric_models/IMUPreintegrator.h"
#include "../geometric_models/IMUModel.h"

namespace libRSF
{
  template<typename ErrorType>
  class IMUPreintegrationFactor : public BaseFactor<ErrorType, true, true, 3, 4, 9, 3, 4, 9>
  {
    public:
      /** construct factor and store measurement */
      IMUPreintegrationFactor(ErrorType &Error, const PreintegratedIMUResult &Preintegration) : Preintegration_(Preintegration)
      {
        /** save noise model */
        this->Error_ = Error;
      }

      /** geometric error model */
      template <typename T>
      VectorT<T, 15> Evaluate(const T* const PointOld, const T* const QuatOld, const T* const SpeedBiasOld,
                              const T* const PointNew, const T* const QuatNew, const T* const SpeedBiasNew) const
      {
        /** map pointer to vectors */
        VectorRefConst<T, 3> P1(PointOld);
        VectorRefConst<T, 3> P2(PointNew);

        VectorRefConst<T, 3> V1(SpeedBiasOld);
        VectorRefConst<T, 3> V2(SpeedBiasNew);

        VectorRefConst<T, 3> BiasAcc1(SpeedBiasOld + 3);
        VectorRefConst<T, 3> BiasAcc2(SpeedBiasNew + 3);

        VectorRefConst<T, 3> BiasTR1(SpeedBiasOld + 6);
        VectorRefConst<T, 3> BiasTR2(SpeedBiasNew + 6);

        QuaternionRefConst<T> Rot1(QuatOld);
        QuaternionRefConst<T> Rot2(QuatNew);

        /** linear approximation for biases */
        const VectorT<T, 3> DeltaBiasAcc = BiasAcc1 - Preintegration_.BiasAcc.template cast<T>();
        const VectorT<T, 3> DeltaBiasTR = BiasTR1 - Preintegration_.BiasTR.template cast<T>();

        const VectorT<T, 3> ApproxTranslation =
            Preintegration_.Translation.template cast<T>() +
            Preintegration_.JacTransBiasAcc.template cast<T>() * DeltaBiasAcc +
            Preintegration_.JacTransBiasTR.template cast<T>() * DeltaBiasTR;

        const VectorT<T, 3> ApproxVelocity =
            Preintegration_.Velocity.template cast<T>() +
            Preintegration_.JacVelBiasAcc.template cast<T>() * DeltaBiasAcc +
            Preintegration_.JacVelBiasTR.template cast<T>() * DeltaBiasTR;

        const QuaternionT<T> ApproxRotation =
            Preintegration_.Rotation.template cast<T>() * QuaternionExpMap<T>(
                Preintegration_.JacRotBiasTRLocal.template cast<T>() * DeltaBiasTR);

        /** calculate error */
        const double dt = Preintegration_.DeltaTime;
        VectorT<T, 15> Error;
        Error.template segment<3>(0) = Rot1.conjugate() * (P2 - P1 - dt * V1 + (0.5*dt*dt * GRAVITY_VECTOR).template cast<T>()) - ApproxTranslation;
        Error.template segment<3>(3) = Rot1.conjugate() * (V2 - V1 + (dt * GRAVITY_VECTOR).template cast<T>()) - ApproxVelocity;
        Error.template segment<3>(6) = QuaternionLogMap<T>(ApproxRotation.conjugate()*Rot1.conjugate()*Rot2);

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
                                           PointNew, QuatNew, SpeedBiasNew),
                            Error);
        return true;
      }

      /** predict the next state for initialization, order is the same as for Evaluate() */
      void predict(const std::vector<double*> &StatePointers) const
      {
        /** get initial states */
        Vector3 P1 = VectorRefConst<double,3>(StatePointers.at(0));
        Quaternion Q1 = QuaternionRefConst<double>(StatePointers.at(1));
        Vector9 SB1 = VectorRefConst<double,9>(StatePointers.at(2));

        /** create intermediate storage */
        Vector3 P2;
        Quaternion Q2;
        Vector9 SB2;

        /** loop over measurements to predict incrementally */
        double OldTime = Preintegration_.StartTime;
        for (const Data &Measurement : Preintegration_.Measurements)
        {
          IMUModel<double>::applyForward(P1.data(),
                                         Q1.coeffs().data(),
                                         SB1.data(),
                                         P2.data(),
                                         Q2.coeffs().data(),
                                         SB2.data(),
                                         Measurement.getMean().head(3),
                                         Measurement.getMean().tail(3),
                                         Measurement.getTimestamp() - OldTime);

          OldTime = Measurement.getTimestamp();
          P1 = P2;
          Q1 = Q2;
          SB1 = SB2;
        }

        /** copy result */
        VectorRef<double,3> P2Ref (StatePointers.at(3));
        QuaternionRef<double> Q2Ref (StatePointers.at(4));
        VectorRef<double,9> SB2Ref (StatePointers.at(5));
        P2Ref = P2;
        Q2Ref = Q2;
        SB2Ref = SB2;
      }

    private:
      const PreintegratedIMUResult Preintegration_;
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template<typename ErrorType>
  struct FactorTypeTranslator<FactorType::IMUPretintegration, ErrorType> {using Type = IMUPreintegrationFactor<ErrorType>;};
}

#endif // IMUPREINTEGRATIONFACTOR_H
