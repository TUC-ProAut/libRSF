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
 * @file ConstantPose2Factor.h
 * @author Tim Pfeifer
 * @date 17.04.2023
 * @brief A Factor that connects two 2D poses with a zero pose.
 * @copyright GNU Public License.
 *
 */

#ifndef CONSTANTPOSE2FACTOR_H
#define CONSTANTPOSE2FACTOR_H

#include "../Geometry.h"
#include "../geometric_models/RelativePoseModel.h"
#include "BaseFactor.h"

namespace libRSF
{
  template <typename ErrorType>
  class ConstantPose2Factor : public BaseFactor<ErrorType, false, false, 2, 1, 2, 1>
  {
   public:
    /** construct factor and store measurement */
    explicit ConstantPose2Factor(ErrorType &Error)
    {
      this->Error_ = Error;
    }

    /** geometric error model */
    template <typename T>
    VectorT<T, 3> Evaluate(const T * const Pos1, const T * const Yaw1, const T * const Pos2, const T * const Yaw2) const
    {
      /** estimate measurements*/
      VectorT<T, 2> TransEst;
      Rotation2DT<T> RotEst;
      RelativePose2Model<T>::applyBackward(Pos1, Yaw1, Pos2, Yaw2, TransEst, RotEst);

      /** error = estimated measurement - measurement */
      VectorT<T, 3> Error;
      Error.template head<2>() = TransEst;
      Error(2) = NormalizeAngle<T>(RotEst.angle());

      return Error;
    }

    /** combine probabilistic and geometric model */
    template <typename T, typename... ParamsType>
    bool operator()(const T * const Pos1, const T * const Yaw1, const T * const Pos2, const T * const Yaw2, ParamsType... Params) const
    {
      return this->Error_.template weight<T>(this->Evaluate(Pos1, Yaw1, Pos2, Yaw2), Params...);
    }

    /** predict the next state for initialization, order is the same as for Evaluate() */
    void predict(const std::vector<double *> &StatePointers) const
    {
      /** wrap reference */
      VectorRefConst<double, 2> const Pos1(StatePointers.at(0));
      VectorRefConst<double, 1> const Yaw1(StatePointers.at(1));
      VectorRef<double, 2> Pos2(StatePointers.at(2));
      VectorRef<double, 1> Yaw2(StatePointers.at(3));

      /** predict */
      Pos2 = Pos1;
      Yaw2 = Yaw1;
    }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template <typename ErrorType>
  struct FactorTypeTranslator<FactorType::LoopPose2, ErrorType>{using Type = ConstantPose2Factor<ErrorType>;};
}  // namespace libRSF

#endif  // CONSTANTPOSE2FACTOR_H
