/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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

#include "factors/MarginalPrior.h"

namespace libRSF
{

  MarginalPrior::MarginalPrior(const std::vector<int> &LocalSize,
                               const std::vector<int> &GlobalSize,
                               const std::vector<Vector> &LinearizationPoints,
                               const std::vector<DataType> & StateTypes,
                               const Matrix &J,
                               const Vector &R)
  {
    /** check if the data is complete */
    if((LocalSize.size() == GlobalSize.size())
       && (LocalSize.size() == LinearizationPoints.size())
       && (LocalSize.size() == StateTypes.size()))
    {
      _LocalSize = LocalSize;
      _GlobalSize = GlobalSize;
      _StateTypes = StateTypes;

      /** compute overall system size */
      int LocalSum = 0, GlobalSum = 0;
      for (int n = 0; n < static_cast<int>(LocalSize.size()); n++)
      {
        LocalSum += LocalSize.at(n);
        GlobalSum += GlobalSize.at(n);
      }
      _LocalSizeSum = LocalSum;
      _GlobalSizeSum = GlobalSum;

      /** store linear system */
      _LinearJacobian = J;
      _LinearResidual = R;

      /** store linearization points */
      _LinearizationPoints.resize(_GlobalSizeSum);
      int CumSum = 0;
      for (int n = 0; n < static_cast<int>(LinearizationPoints.size()); n++)
      {
        _LinearizationPoints.segment(CumSum, GlobalSize.at(n)) = LinearizationPoints.at(n);
        CumSum += GlobalSize.at(n);
      }

      /** parametrize factor */
      this->set_num_residuals(_LocalSizeSum);
      for (int BlockSize : GlobalSize)
      {
        this->mutable_parameter_block_sizes()->push_back(BlockSize);
      }
    }
    else
    {
      PRINT_ERROR("Marginal data not correct!");
    }
  }

  bool MarginalPrior::Evaluate(double const* const* Parameters,
                               double* Residuals,
                               double** Jacobians) const
  {

    /** use seperate jacobian to represent the manifold operations */
    Matrix JacobianManifold;
    bool HasJacobian = false;
    if(Jacobians != nullptr)
    {
      JacobianManifold.resize(_LocalSizeSum, _GlobalSizeSum);
      JacobianManifold.setZero();
      HasJacobian = true;
    }

    VectorRef<double, Dynamic> Error(Residuals, _LocalSizeSum);
    Vector DeltaState(_LocalSizeSum);

    /** compute blockwise error */
    int IndexError = 0;
    int IndexState = 0;
    for (int nState = 0; nState < static_cast<int>(_GlobalSize.size()); nState++)
    {
      /** get dimensions of sub-block */
      int GlobalSize = _GlobalSize.at(nState);
      int LocalSize = _LocalSize.at(nState);

      /** map relevant variables */
      const VectorRefConst<double, Dynamic> State(Parameters[nState], GlobalSize);
      const Vector LinearState = _LinearizationPoints.segment(IndexState, GlobalSize);

      if (_StateTypes.at(nState) == DataType::Angle)
      {
        /** angle case */
        DeltaState.segment(IndexError, LocalSize) = NormalizeAngleVector<double, 1>(State - LinearState);

        /** identity jacobian */
        if(HasJacobian)
        {
          JacobianManifold.block(IndexError, IndexState, LocalSize, GlobalSize).setIdentity();
        }

      }
      else if (_StateTypes.at(nState) == DataType::Quaternion)
      {
        /** quaternion case */
        QuaternionRefConst<double> QState (State.data());
        QuaternionRefConst<double> QPrior (LinearState.data());

        if(HasJacobian)
        {
          /** tangent space error and jacobian */
          Matrix34 JacState;
          DeltaState.segment(IndexError, LocalSize) = QuaternionError(QState, QPrior, &JacState, nullptr);
          JacobianManifold.block(IndexError, IndexState, LocalSize, GlobalSize) = JacState;
        }
        else
        {
          /** tangent space error */
          DeltaState.segment(IndexError, LocalSize) = QuaternionError<double>(QState, QPrior);
        }
      }
      else
      {
        if(LocalSize != GlobalSize)
        {
          PRINT_ERROR("You are using a local parametrization that is not handled here! We disable the marginal prior for this error block!");

          /** disable error block */
          DeltaState.segment(IndexError, LocalSize).setZero();
        }
        else
        {
          /** normal case */
          DeltaState.segment(IndexError, LocalSize) = State - LinearState;
        }

        /** identity jacobian */
        if(HasJacobian)
        {
          JacobianManifold.block(IndexError, IndexState, LocalSize, GlobalSize).setIdentity();
        }
      }

      /** move index to the next error block*/
      IndexError += LocalSize;
      IndexState += GlobalSize;
    }

    /** apply linear jacobian and residual */
    Error = _LinearJacobian * DeltaState + _LinearResidual;

    /** jacobians are composed of linear jacobian and manifold jacobian */
    if(HasJacobian)
    {
      int IndexStateJac = 0;
      for (int nState = 0; nState < static_cast<int>(_GlobalSize.size()); nState++)
      {
        int GlobalSize = _GlobalSize.at(nState);

        if(Jacobians[nState])
        {
          MatrixRef<double, Dynamic, Dynamic> Jacobian(Jacobians[nState], _LocalSizeSum, GlobalSize);
          Jacobian = _LinearJacobian * JacobianManifold.middleCols(IndexStateJac, GlobalSize);
        }

        IndexStateJac += GlobalSize;
      }
    }

    return true;
  }

}
