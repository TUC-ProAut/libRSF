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
 * @file SwitchableConstraints.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief Error functions for the Switchable Constraints algorithm by Sünderhauf et al.
 * @copyright GNU Public License.
 *
 */

#ifndef SWITCHABLECONSTRAINTS_H
#define SWITCHABLECONSTRAINTS_H

#include "ErrorModel.h"
#include "../VectorMath.h"

namespace libRSF
{
  /** \brief The robust Switchable Constraints error model
   * Based on:
   * N. Sünderhauf and P. Protzel
   * “Switchable constraints for robust pose graph SLAM”
   * in Proc. of Intl. Conf. on Intelligent Robots and Systems (IROS), Vilamoura, 2012.
   * DOI: 10.1109/IROS.2012.6385590
   *
   * \param BaseModelType Underlying non-robust error model (most probably Gaussian)
   * \param Sigma Tuning parameter of SC
   *
   */
  template <int Dim, typename BaseModelType>
  class SwitchableConstraints : public ErrorModel <Dim, Dim+1, 1>
  {
  public:

      SwitchableConstraints() = default;

      explicit SwitchableConstraints(BaseModelType BaseModel, double Sigma): Sigma_(Sigma), BaseModel_(BaseModel)
      {}

      void setSigma(double Sigma)
      {
        Sigma_ = Sigma;
      }

      template <typename T>
      bool weight(const VectorT<T, Dim> &RawError, const T* const SwitchVariable, T* Error) const
      {
        if (this->Enable_)
        {
          /** evaluate with the non-robust model */
          BaseModel_.weight(RawError, Error);

          /** store error in matrix */
          VectorRef<T, Dim+1> ErrorMap(Error);

          /** apply switch variable */
          ErrorMap.template head<Dim>().array() *= SwitchVariable[0];

          /** add switch prior */
          ErrorMap(Dim) = (SwitchVariable[0] - 1.0) / Sigma_;
        }
        else
        {
          /** pass raw error trough */
          VectorRef<T, Dim> ErrorMap(Error);
          ErrorMap = RawError;

          /** set unused dimension to 0 */
          Error[Dim] = T(0.0);
        }

        return true;
      }

  private:
    double Sigma_ = 1.0;
    BaseModelType BaseModel_;
  };
}


#endif // SWITCHABLECONSTRAINTS_H
