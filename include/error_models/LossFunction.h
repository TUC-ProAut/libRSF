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
 * @file LossFunction.h
 * @author Tim Pfeifer
 * @date 22.11.2016
 * @brief Collection of classes that represent robust cost functions
 * @copyright GNU Public License.
 *
 */

#ifndef LOSSFUNCTION_H
#define LOSSFUNCTION_H

#include <ceres/ceres.h>

namespace libRSF
{
  /** inherit existing loss functions */
  using ceres::HuberLoss;
  using ceres::TukeyLoss;
  using ceres::CauchyLoss;
  using ceres::SoftLOneLoss;

  /** \brief The robust Dynamic Covariance Scaling loss function
   * Based on:
   * P. Agarwal, G. D. Tipaldi, L. Spinello, C. Stachniss and W. Burgard
   * "Robust map optimization using dynamic covariance scaling"
   * IEEE International Conference on Robotics and Automation, Karlsruhe, 2013
   * DOI: 10.1109/ICRA.2013.6630557
   *
   * \param Phi Tuning parameter of DCS
   *
   */
  class DCSLoss : public ceres::LossFunction
  {
  public:
      explicit DCSLoss(double Phi) : Phi_(Phi) {};
      ~DCSLoss() override = default;

      void Evaluate(double, double*) const override;

  private:
    const double Phi_;
  };

  /** \brief The robust closed form of Dynamic Covariance Estimation
   * Based on:
   * T. Pfeifer, S. Lange and P. Protzel
   * "Dynamic Covariance Estimation — A parameter free approach to robust Sensor Fusion"
   * IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI), Daegu, 2017
   * DOI: 10.1109/MFI.2017.8170347
   *
   * \param Sigma Standard deviation without outliers
   *
   */
  class cDCELoss : public ceres::LossFunction
  {
  public:
      explicit cDCELoss(double Sigma) : Sigma_(Sigma) {};
      ~cDCELoss() override = default;

      void Evaluate(double, double*) const override;

  private:
    const double Sigma_;
  };

  /** \brief M-estimator representing a Student's t-distribution
   * \param Nu Degree of freedom parameter
   * \param Dim Number of Dimensions
   */
  class StudentLoss : public ceres::LossFunction
  {
  public:
      explicit StudentLoss(const double Nu, const double Dim) : Nu_(Nu), Dim_(Dim) {};
      ~StudentLoss() override = default;

      void Evaluate(double, double*) const override;

  private:
    const double Nu_;
    const double Dim_;
  };

  /** \brief M-estimator exactly representing a Cauchy distribution
   * Please note that the ceres::CauchyLoss is a heuristic function:
   * https://groups.google.com/g/ceres-solver/c/RXyOqy_n0p8
   * \param Scale Scale parameter of the distribution
   */
  class CauchyPDFLoss : public ceres::LossFunction
  {
  public:
      explicit CauchyPDFLoss(const double Scale) : c_(Scale*Scale) {};
      ~CauchyPDFLoss() override = default;

      void Evaluate(double, double*) const override;

  private:
    const double c_;
  };

  /** \brief M-estimator representing a general adaptive loss function
   * Based on:
   * Jonathan T. Barron
   * "A General and Adaptive Robust Loss Function"
   * IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), Long Beach, 2019
   * DOI: 10.1109/CVPR.2019.00446
   *
   * \param Alpha adapts the kernel shape
   * \param C Scaling of the inliers, equivalent to a standard deviation
   */
  class GeneralAdaptiveLoss : public ceres::LossFunction
  {
  public:
      explicit GeneralAdaptiveLoss(const double Alpha, const double C = 1.0) : Alpha_(Alpha), Cov_(C*C) {};
      ~GeneralAdaptiveLoss() override = default;

      void Evaluate(double, double*) const override;

  private:
    const double Alpha_;
    const double Cov_;
  };
}

#endif // LOSSFUNCTION_H
