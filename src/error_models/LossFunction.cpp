/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
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

#include "error_models/LossFunction.h"

namespace libRSF
{
  /** @brief implements the original DCS loss function
   *
   * @param s   squared non-weighted error
   * @param rho [p(s) p'(s) p''(s)]
   */
  void DCSLoss::Evaluate(double s, double rho[3]) const
  {
    if (s <= Phi_)
    {
      rho[0] = s;
      rho[1] = 1;
      rho[2] = 0;
    }
    else
    {
      rho[0] = Phi_ * (3 * s - Phi_) / (s + Phi_);
      rho[1] = (3*Phi_)/(Phi_ + s) + (Phi_*(Phi_ - 3*s))/pow((Phi_ + s), 2);
      rho[2] = - (6*Phi_)/pow((Phi_ + s), 2) - (2*Phi_*(Phi_ - 3*s))/pow((Phi_ + s), 3);
    }
  }

  /** @brief implements the a new DCS like loss function which is closer related to the mathematical meaning of a gauss distribution
   *
   * @param s   squared non-weighted error
   * @param rho [p(s) p'(s) p''(s)]
   */
  void cDCELoss::Evaluate(double s, double rho[3]) const
  {
    double e = sqrt(s);

    if (e <= Sigma_)
    {
      rho[0] = s / (Sigma_ * Sigma_);
      rho[1] = 1.0 / (Sigma_ * Sigma_);
      rho[2] = 0.0;
    }
    else
    {
      rho[0] = 1.0 + log(e / Sigma_) * 2;
      rho[1] = 1.0 / (s);
      rho[2] = -1.0 / (s * s);
    }
  }
}
