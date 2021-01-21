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

#include "../VectorMath.h"
#include "../Messages.h"

#include <cmath>

namespace libRSF
{

  template <int Dim>
  class GaussianComponent
  {
    public:
      GaussianComponent()
      {
        _Scaling = 1;
      }

      ~GaussianComponent() = default;

      typedef MatrixStatic<Dim, Dynamic> ErrorMatType;

      void setParamsStdDev(const MatrixStatic<Dim, 1> &StdDev,
                           const MatrixStatic<Dim, 1> &Mean,
                           const MatrixStatic<1, 1> &Weight)
      {
        _Mean = Mean;
        _Weight = Weight;

        /** write information matrix directly */
        _SqrtInformation = StdDev.cwiseInverse().asDiagonal();

        updateScaling();
      }

      void setParamsCovariance(const MatrixStatic<Dim, Dim> &Covariance,
                               const MatrixStatic<Dim, 1> &Mean,
                               const MatrixStatic<1, 1> &Weight)
      {
        _Mean = Mean;
        _Weight = Weight;

        /** use decomposition to get square root information matrix */
        _SqrtInformation = InverseSquareRoot(Covariance);

        updateScaling();
      }

      void setParamsInformation(const MatrixStatic<Dim, Dim> &Information,
                                const MatrixStatic<Dim, 1> &Mean,
                                const MatrixStatic<1, 1> &Weight)
      {
        _Mean = Mean;
        _Weight = Weight;
        _SqrtInformation = SquareRoot(Information);

        updateScaling();
      }

      void setParamsSqrtInformation(const MatrixStatic<Dim, Dim> &SqrtInformation,
                                    const MatrixStatic<Dim, 1> &Mean,
                                    const MatrixStatic<1, 1> &Weight)
      {
        _Mean = Mean;
        _Weight = Weight;
        _SqrtInformation = SqrtInformation;

        updateScaling();
      }

      void setSqrtInformation(const MatrixStatic<Dim, Dim> &SqrtInformation)
      {
        _SqrtInformation = SqrtInformation;

        updateScaling();
      }

      void updateCovariance(const MatrixStatic<Dim, Dim> &Covariance)
      {
        _SqrtInformation = InverseSquareRoot(Covariance);

        updateScaling();
      }

      MatrixStatic<Dim, 1> getMean() const
      {
        return _Mean;
      }

      MatrixStatic<1, 1> getWeight() const
      {
        return _Weight;
      }

      MatrixStatic<Dim, Dim> getSqrtInformation() const
      {
        return _SqrtInformation;
      }

      MatrixStatic<Dim, Dim> getCovariance() const
      {
        return (_SqrtInformation.transpose() * _SqrtInformation).inverse();
      }

      void setMean(MatrixStatic<Dim, 1> Mean)
      {
        _Mean = Mean;
      }

      /** return the part inside the exp() function */
      template <typename T>
      MatrixT<T, Dim, 1 > getExponentialPart(const VectorT<T, Dim> &Error) const
      {
        VectorT<T, Dim> WeightedError;

        /** shift by mean */
        WeightedError = Error + _Mean.template cast<T>();

        /** scale with information matrix */
        WeightedError = _SqrtInformation.template cast<T>() * WeightedError;

        return WeightedError;
      }

      /** return the part before the exp() function */
      template <typename T>
      double getLinearPart(const VectorT<T, Dim> &Error) const
      {
        return _Scaling;
      }

      /** return the maximum value at the mode of the probability function*/
      double getMaximum() const
      {
        return _Scaling;
      }

      /** compute probability for E step of EM */
      Vector computeLikelihood (const ErrorMatType &Errors) const
      {
        /** apply mean */
        ErrorMatType WeightedError = (Errors.array().colwise() + _Mean.array()).matrix();

        /** multiply each row with square root information */
        for (Index n = 0; n < WeightedError.cols(); ++n)
        {
          WeightedError.col(n) = (WeightedError.col(n).transpose() * _SqrtInformation).transpose();
        }

        return ((WeightedError.array().square().colwise().sum() / -2.0).exp() * _Scaling).matrix();
      }

      /** estimate parameters for M step of EM */
      void estimateParameters (const ErrorMatType &Errors,
                               const Vector &Likelihoods,
                               bool  EstimateMean)
      {
        const double LikelihoodSum = Likelihoods.sum();

        /** estimate weight */
        _Weight(0) = LikelihoodSum / Likelihoods.rows();

        /** estimate mean */
        if (EstimateMean == true)
        {
           _Mean = -((Errors.array().rowwise() * Likelihoods.transpose().array()).rowwise().sum() / LikelihoodSum).matrix();
        }

        /** estimate covariance */
        MatrixStatic<Dim,Dim> Covariance = MatrixStatic<Dim,Dim>::Zero();
        for (Index n = 0; n < Errors.cols(); ++n)
        {
          MatrixStatic<Dim, 1> Diff = Errors.col(n) + _Mean;
          Covariance += Diff * Diff.transpose() * Likelihoods(n);
        }
        Covariance.array() /= LikelihoodSum;
        _SqrtInformation = InverseSquareRoot(Covariance);

        /** check for degenerated square root information matrix */
        if(_SqrtInformation.array().isFinite().all() == false)
        {
          /** disable component if degenerated */
          _Weight.setZero();
          _Mean.setZero();
          _SqrtInformation = MatrixStatic<Dim, Dim>::Identity();
        }

        /** check for extreme numerical values */
        if((_SqrtInformation.array() > 1e10).any() == true)
        {
          #ifndef NDEBUG
          PRINT_WARNING("Square root information is too high: ", _SqrtInformation.maxCoeff());
          PRINT_WARNING("Limit Square root information to 1e10 for numerical reasons!");
          #endif
          _SqrtInformation = (_SqrtInformation.array() < 1e10).select(_SqrtInformation, 1e10);
        }

        updateScaling();
      }

      void estimateParametersMAP (const ErrorMatType &Errors,
                                  const Vector &Likelihoods,
                                  double DirichletConcentration,
                                  double DirichletConcentrationSum,
                                  double NormalInfoScaling,
                                  VectorStatic<Dim> NormalMean,
                                  MatrixStatic<Dim,Dim> WishartScalingMatrix,
                                  double WishartDOF,
                                  bool EstimateMean)
      {
        /** precalculate shared values */
        const double LikelihoodSum = Likelihoods.sum();

        /** estimate weight */
        _Weight(0) = (DirichletConcentration - 1.0 + LikelihoodSum)
                     /
                     (DirichletConcentrationSum + Likelihoods.rows());

        /** estimate mean */
        if (EstimateMean == true)
        {
          VectorStatic<Dim> WeightedError = (Errors.array().rowwise() * Likelihoods.transpose().array()).rowwise().sum();
          _Mean = -((NormalMean*NormalInfoScaling) + WeightedError)
                  /
                  (NormalInfoScaling + LikelihoodSum);
        }

        /** estimate covariance */
        MatrixStatic<Dim,Dim> Covariance = WishartScalingMatrix + NormalInfoScaling * (NormalMean - _Mean)*(NormalMean - _Mean).transpose();
        for (Index n = 0; n < Errors.cols(); ++n)
        {
          MatrixStatic<Dim, 1> Diff = Errors.col(n) + _Mean;
          Covariance += Diff * Diff.transpose() * Likelihoods(n);
        }
        Covariance.array() /= (WishartDOF - Dim + LikelihoodSum);
        _SqrtInformation = InverseSquareRoot(Covariance);

        /** check for degenerated square root information matrix */
        if(_SqrtInformation.array().isFinite().all() == false)
        {
          /** disable component if degenerated */
          _Weight.setZero();
          _Mean.setZero();
          _SqrtInformation = MatrixStatic<Dim, Dim>::Identity();
        }

        updateScaling();
      }

      /** compute negative log-likelihood of a sample vactor */
      Vector computeNegLogLikelihood (const ErrorMatType &Errors) const
      {
        /** apply mean */
        ErrorMatType WeightedError = (Errors.array().colwise() + _Mean.array()).matrix();

        /** multiply each row with square root information */
        for (int n = 0; n < WeightedError.cols(); ++n)
        {
          WeightedError.col(n) = (WeightedError.col(n).transpose() * _SqrtInformation).transpose();
        }

        return ((WeightedError.array().square().colwise().sum() / 2.0) - log(_Scaling)).matrix();
      }

      bool checkParameters () const
      {
        bool Passed = true;

        /** check weight */
        if(_Weight.allFinite() == false)
        {
          PRINT_WARNING("Weight is not finite!");
          Passed = false;
        }
        else if(_Weight(0) <= 0)
        {
          PRINT_WARNING("Weight is not positive!");
          Passed = false;
        }
        else if(_Weight(0) > 1)
        {
          PRINT_WARNING("Weight is bigger than one!");
          Passed = false;
        }

        /** check mean */
        if(_Mean.allFinite() == false)
        {
          PRINT_WARNING("Mean is not finite!");
          Passed = false;
        }

        /** check uncertainty */
        if(_SqrtInformation.allFinite() == false)
        {
          PRINT_WARNING("Square root information is not finite!");
          Passed = false;
        }
        else
        {
          if(IsPositiveSemidefinite<Dim>(_SqrtInformation) == false)
          {
            PRINT_WARNING("Square root information is not positive semi-definite!");
            Passed = false;
          }
        }

        /** check scaling */
        if(std::isfinite(_Scaling) == false)
        {
          PRINT_WARNING("Scaling is not finite!");
          Passed = false;
        }

        return Passed;
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

      MatrixStatic<Dim, 1> _Mean;
      MatrixStatic<1, 1> _Weight;
      MatrixStatic<Dim, Dim> _SqrtInformation;
      double _Scaling;
  };

  /** compare functions for ordering */
  template <int Dim>
  bool compareByMode(const GaussianComponent<Dim> & a, const GaussianComponent<Dim> & b)
  {
    return (a.getWeight()(0) * a.getSqrtInformation().determinant()) > (b.getWeight()(0) * b.getSqrtInformation().determinant());
  }

  template <int Dim>
  bool compareByMean(const GaussianComponent<Dim> & a, const GaussianComponent<Dim> & b)
  {
    return a.getMean().sum() < b.getMean().sum();
  }

  template <int Dim>
  bool compareByAbsMean(const GaussianComponent<Dim> & a, const GaussianComponent<Dim> & b)
  {
    return std::abs(a.getMean().sum()) < std::abs(b.getMean().sum());
  }

  template <int Dim>
  bool compareByWeight(const GaussianComponent<Dim> & a, const GaussianComponent<Dim> & b)
  {
    return a.getWeight()(0) > b.getWeight()(0);
  }

  template <int Dim>
  bool compareByLOSness(const GaussianComponent<Dim> & a, const GaussianComponent<Dim> & b)
  {
    return (a.getWeight()(0) * a.getSqrtInformation().determinant() / (std::abs(a.getMean().sum()) + 1e-6))
           >
           (b.getWeight()(0) * b.getSqrtInformation().determinant() / (std::abs(b.getMean().sum()) + 1e-6));
  }

  /** Bhattacharyya distance for merging */
  template <int Dim>
  double CalculateBhattacharyyaDistance(const GaussianComponent<Dim> & Gaussian1, const GaussianComponent<Dim> & Gaussian2)
  {
    MatrixStatic<Dim, Dim> Cov1 = Gaussian1.getCovariance();
    MatrixStatic<Dim, Dim> Cov2 = Gaussian2.getCovariance();
    MatrixStatic<Dim, Dim> CovMean = (Cov1 + Cov2).array() / 2.0;

    MatrixStatic<Dim, 1> MeanDiff = Gaussian1.getMean() - Gaussian2.getMean();

    MatrixStatic<1, 1> Mahala = 0.125 * MeanDiff.transpose() * CovMean.inverse() * MeanDiff;

    return Mahala(0) + 0.5 * log(CovMean.determinant() / sqrt(Cov1.determinant()*Cov2.determinant()));
  }

  /** merge two gaussians */
  template <int Dim>
  GaussianComponent<Dim> MergeGaussians (const GaussianComponent<Dim> &Gaussian1, const GaussianComponent<Dim> &Gaussian2)
  {
    MatrixStatic<1, 1> Weight12 = Gaussian1.getWeight() + Gaussian2.getWeight();
    MatrixStatic<Dim, 1> Mean12 = (Gaussian1.getMean() * Gaussian1.getWeight() + Gaussian2.getMean() * Gaussian2.getWeight()).array() / Weight12(0);

    MatrixStatic<Dim, Dim> SqrtInfo12 = (Gaussian1.getSqrtInformation() * Gaussian1.getWeight()(0) + Gaussian2.getSqrtInformation() * Gaussian2.getWeight()(0)).array() / Weight12(0);

    GaussianComponent<Dim> Gaussian12;
    Gaussian12.setParamsSqrtInformation(SqrtInfo12, Mean12, Weight12);

    return Gaussian12;
  }

}

#endif // GAUSSIANCOMPONENT_H
