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

/**
 * @file GaussianComponent.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A single Gaussian component to compose mixtures. Not intended for standanlone usage.
 * @copyright GNU Public License.
 *
 */

#ifndef GAUSSIANCOMPONENT_H
#define GAUSSIANCOMPONENT_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace libRSF
{
  template <int Dimension>
  class GaussianComponent
  {
    public:
      GaussianComponent()
      {
        _Scaling = 1;
      }

      ~GaussianComponent() {};

      void setParamsStdDev(ceres::Vector StdDev, ceres::Vector Mean, ceres::Vector Weight)
      {
        _Mean = Mean;
        _Weight = Weight;

        /** write information matrix directly */
        _SqrtInformation = StdDev.cwiseInverse().asDiagonal();

        updateScaling();
      }

      Eigen::Matrix<double, Dimension, 1> getMean() const
      {
        return _Mean;
      }

      Eigen::Matrix<double, 1, 1> getWeight() const
      {
        return _Weight;
      }

      Eigen::Matrix<double, Dimension, Dimension> getSqrtInformation() const
      {
        return _SqrtInformation;
      }

      void setMean(Eigen::Matrix<double, Dimension, 1> Mean)
      {
        _Mean = Mean;
      }

      /** return the part inside the exp() function */
      template <typename T>
      Eigen::Matrix < T, Dimension, 1 > getExponentialPart(T * const Error) const
      {
        Eigen::Map <Eigen::Matrix<T, Dimension, 1> > ErrorMap(Error);
        Eigen::Matrix <T, Dimension, 1> WeightedError;

        /** shift by mean */
        WeightedError = ErrorMap + _Mean.template cast<T>();
        /** scale with information matrix */
        WeightedError = _SqrtInformation.template cast<T>() * WeightedError;

        return WeightedError;
      }

      /** return the part before the exp() function */
      template <typename T>
      double getLinearPart(T * const Error) const
      {
        return _Scaling;
      }

      /** return the maximum value at the mode of the probability function*/
      double getMaximum() const
      {
        return _Scaling;
      }

      /** compute probability for E step of EM */
      Eigen::VectorXd computeProbability (Eigen::Matrix<double, Eigen::Dynamic, Dimension> const &Errors) const
      {
        /** apply mean */
        Eigen::Matrix<double, Eigen::Dynamic, Dimension> WeightedError = (Errors.array().rowwise() + _Mean.transpose().array()).matrix();

        /** multiply each row with square root information */
        for (size_t n = 0; n < WeightedError.rows(); ++n)
        {
          WeightedError.row(n) = WeightedError.row(n) * _SqrtInformation;
        }

        return ((WeightedError.array().square().rowwise().sum() / -2.0).exp() * _Scaling).matrix();
      }

      /** estimate parameters for M step of EM */
      void estimateParameters (Eigen::Matrix<double, Eigen::Dynamic, Dimension> const  &Errors,
                               Eigen::Matrix<double, Eigen::Dynamic, 1> const  &Likelihoods,
                               bool KeepMean)
      {
        double LikelihoodSum = Likelihoods.sum();

        _Weight(0) = LikelihoodSum / Likelihoods.rows();

        if (KeepMean == false)
        {
          _Mean = -((Errors.array().colwise() * Likelihoods.array()).colwise().sum() / LikelihoodSum).matrix().transpose();
        }

        Eigen::Matrix<double, Dimension, Dimension> Covariance = Eigen::Matrix<double, Dimension, Dimension>::Zero();
        for (size_t n = 0; n < Errors.rows(); ++n)
        {
          Covariance += (Errors.row(n) + _Mean).transpose() * (Errors.row(n) + _Mean) * Likelihoods(n);
        }
        Covariance.array() /= LikelihoodSum;
        _SqrtInformation = Covariance.inverse().llt().matrixL();

        /** check for degenerated SqrtInfo matrix */
        if(!(_SqrtInformation.array().isFinite().all()))
        {
          /** disable component if degenerated */
          _Weight = Eigen::Matrix<double, 1, 1>::Zero();
          _Mean = Eigen::Matrix<double, Dimension, 1>::Zero();
          _SqrtInformation = Eigen::Matrix<double, Dimension, Dimension>::Identity();
        }

        updateScaling();
      }

    private:
      void updateScaling()
      {
        _Scaling = calculateNormalization();
      }

      double calculateNormalization()
      {
        return _Weight(0, 0) * _SqrtInformation.determinant();
      }


      Eigen::Matrix<double, Dimension, 1> _Mean;
      Eigen::Matrix<double, 1, 1> _Weight;
      Eigen::Matrix<double, Dimension, Dimension> _SqrtInformation;
      double _Scaling;
  };

  /** compare functions for ordering */
  template <int Dimension>
  bool compareByMean(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return a.getMean().sum() < b.getMean().sum();
  }

  template <int Dimension>
  bool compareByWeight(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return a.getWeight()(0) > b.getWeight()(0);
  }



}

#endif // GAUSSIANCOMPONENT_H
