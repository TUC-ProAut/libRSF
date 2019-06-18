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
 * @file GaussianMixture.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A weighted mixture of multiple Gaussians. Not intended for standalone usage.
 * @copyright GNU Public License.
 *
 */

#ifndef GAUSSIANMIXTURE_H
#define GAUSSIANMIXTURE_H

#include <algorithm>
#include <ceres/ceres.h>

#include <unsupported/Eigen/SpecialFunctions>

#include "GaussianComponent.h"
#include "../StateData.h"


namespace libRSF
{
  template <int Dimension>
  class GaussianMixture
  {
    public:
      GaussianMixture() {};
      virtual ~GaussianMixture() {};

      /** only for 1D GMMs */
      GaussianMixture(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight);
      void addDiagonal(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight);

      void addComponent(GaussianComponent<Dimension> Gaussian)
      {
        _Mixture.emplace_back(Gaussian);
      }

      void removeLastComponent()
      {
        _Mixture.pop_back();
      }

      void clear()
      {
        _Mixture.clear();
      }

      size_t getNumberOfComponents() const
      {
        return _Mixture.size();
      }

      typedef Eigen::Matrix<double, Eigen::Dynamic, Dimension> ErrorMatType;

      double computeLikelihood(ErrorMatType &DataVector, Eigen::MatrixXd &Likelihood)
      {
        size_t M = Likelihood.cols();

        /** E-step */
        for(size_t m = 0; m < M; ++m)
        {
          Likelihood.col(m) = _Mixture.at(m).computeProbability(DataVector);
        }

        /** remove NaNs */
        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);

        /** calculate relative likelihood  */
        double LikelihoodSumNew = Likelihood.sum();
        Eigen::VectorXd LikelihoodRowSum = Likelihood.rowwise().sum();

        for(size_t m = 0; m < M; ++m)
        {
          Likelihood.col(m).array() /= LikelihoodRowSum.array();
        }

        /** remove NaNs (occur if sum of likelihoods is zero) */
        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 1.0/M);

        return LikelihoodSumNew;
      }

      void computeMixtureParameters(ErrorMatType &DataVector, Eigen::MatrixXd &Likelihood)
      {
        size_t M = Likelihood.cols();

        for(size_t m = 0; m < M; ++m)
        {
          _Mixture.at(m).estimateParameters(DataVector,Likelihood.col(m),false);
        }
      }

      bool estimateWithEM(std::vector<double> &Data, bool Adaptive = false)
      {
        size_t N = Data.size();
        size_t M = _Mixture.size();

        /** map data to eigen vector */
        ErrorMatType DataVector = Eigen::Map<const ErrorMatType, Eigen::Unaligned>(Data.data(), N, Dimension);

        /** convergence criteria*/
        double LikelihoodSumOld = 1e40;
        double LikelihoodSumNew;
        bool PerfomedRemove = false;
        bool ReachedConvergence = false;

        /** N x M matrix */
        Eigen::MatrixXd Likelihood;

        /** repeat until convergence or 200 iterations*/
        for(int i = 0; i < 200; ++i)
        {
          M = _Mixture.size();
          Likelihood.resize(N, M);
          PerfomedRemove = false;

          /** E-step (multi-threaded) */
          LikelihoodSumNew = computeLikelihood(DataVector, Likelihood);

          /** M-Step (multi-threaded) */
          computeMixtureParameters(DataVector, Likelihood);

          double LikelihoodChange = std::abs(LikelihoodSumOld - LikelihoodSumNew)/LikelihoodSumNew;

          if(Adaptive)
          {
            for(size_t m = 0; m < M; ++m)
            {
              if (_Mixture.at(m).getWeight()(0) < (Dimension+1.0)/N)
              {
                _Mixture.erase(_Mixture.begin() + m);
                M -= 1;
                PerfomedRemove = true;
              }
            }
          }

          /** check for convergence */
          if(LikelihoodChange < 1e-5)
          {
            ReachedConvergence = true;
          }
          else
          {
            ReachedConvergence = false;
          }

          /** terminate loop */
          if (ReachedConvergence && !PerfomedRemove)
          {
            return true;
          }

          LikelihoodSumOld = LikelihoodSumNew;
        }

        return false;
      }

      /** variational bayesian inference */
      void estimateWithVBI(std::vector<double> &Data, double Nu);

      void printParameter()
      {
        std::cout << "Mean" <<' ' << "StdDev" <<' ' << "Weight" <<std::endl;
        for(size_t n = 0; n < _Mixture.size(); ++n)
        {
          std::cout << _Mixture.at(n).getMean() <<' ' << _Mixture.at(n).getSqrtInformation().inverse() <<' ' << _Mixture.at(n).getWeight() <<std::endl;
        }
      }

      /** query error values for a specific component */
      template <typename T>
      Eigen::Matrix < T, Dimension, 1 > getExponentialPartOfComponent(size_t NumberOfComponent, T * const Error) const
      {
        return _Mixture.at(NumberOfComponent-1).getExponentialPart(Error);
      }

      template <typename T>
      double getLinearPartOfComponent(size_t NumberOfComponent, T * const Error) const
      {
        return _Mixture.at(NumberOfComponent-1).getLinearPart(Error);
      }

      double getMaximumOfComponent(size_t NumberOfComponent) const
      {
        return _Mixture.at(NumberOfComponent-1).getMaximum();
      }

      /** special function for pseudo range stuff */
      Eigen::VectorXd removeOffset();
      void removeGivenOffset(Eigen::VectorXd Offset);

      /** sort for better readability */
      void sortComponentsByMean()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByMean<Dimension>);
      }

      void sortComponentsByWeight()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByWeight<Dimension>);
      }

      private:
        std::vector<GaussianComponent<Dimension>> _Mixture;
  };
}

#endif // GAUSSIANMIXTURE_H
