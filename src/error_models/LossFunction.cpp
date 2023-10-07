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

#include "error_models/LossFunction.h"

namespace libRSF
{
  /** @brief Implements the original DCS loss function
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

  /** @brief Implements a new DCS like loss function which is closer related to the mathematical meaning of a Gaussian distribution
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

  /** @brief Implements a M-estimator that represents a Student's t-distribution
   *
   * @param s   squared non-weighted error
   * @param rho [p(s) p'(s) p''(s)]
   */
  void StudentLoss::Evaluate(double s, double rho[3]) const
  {
    rho[0] = (Nu_ + Dim_) * log(1 + s/Nu_);
    rho[1] = (Nu_ + Dim_) / (Nu_ + s);
    rho[2] = -(Nu_ + Dim_) / (Nu_ + s)*(Nu_ + s);
  }

  /** @brief Implements a M-estimator that represents a Cauchy distribution
   *
   * @param s   squared non-weighted error
   * @param rho [p(s) p'(s) p''(s)]
   */
  void CauchyPDFLoss::Evaluate(double s, double rho[3]) const
  {
    rho[0] = 2 * log(1 + s/c_);
    rho[1] = 2 / (c_ + s);
    rho[2] = -2 / pow(c_ + s, 2);
  }

  /** @brief Implements a M-estimator that represents a general adaptive loss function
   *
   * @param s   squared non-weighted error
   * @param rho [p(s) p'(s) p''(s)]
   */
  void GeneralAdaptiveLoss::Evaluate(double s, double rho[3]) const
  {
    if(Alpha_ == 2.0)
    {
      rho[0] = s/Cov_;
      rho[1] = Cov_;
      rho[2] = 0.0;
    }
    else if (Alpha_ == 0.0)
    {
      rho[0] = 2 * log(0.5 * s/Cov_ + 1);
      rho[1] = 1 / (Cov_ + 0.5*s);
      rho[2] = - 0.5 / pow(Cov_ + 0.5*s, 2);
    }
    else if (Alpha_ < -1e10)/**< this approximates negative infinity */
    {
      rho[0] = 2 * (1 - exp(-0.5 * s/Cov_));
      rho[1] = exp(-0.5 * s/Cov_) / Cov_;
      rho[2] = -0.5 * exp(-0.5 * s/Cov_) / pow(Cov_,2);
    }
    else
    {
      rho[0] = 2 * abs(Alpha_ - 2) / Alpha_  * (pow(s/Cov_ / abs(Alpha_ - 2) + 1, Alpha_/2) - 1);
      rho[1] = pow(s/Cov_ / abs(Alpha_ - 2) + 1, Alpha_/2 - 1) / Cov_;
      rho[2] = (Alpha_/2 - 1) * pow(s/Cov_ / abs(Alpha_ - 2) + 1, Alpha_/2 - 2) / (pow(Cov_,2) * abs(Alpha_ - 2));
    }
  }
}
