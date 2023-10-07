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
 * @file GaussianComponent.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A single Gaussian component to compose mixtures. Not intended for standalone usage.
 * @copyright GNU Public License.
 *
 */

#ifndef GAUSSIANCOMPONENT_H
#define GAUSSIANCOMPONENT_H

#include "../VectorMath.h"
#include "../Messages.h"

#include <cmath>
#include <random>

namespace libRSF
{

  template <int Dim>
  class GaussianComponent
  {
    public:
      GaussianComponent()
      {
        Scaling_ = 1;
      }

      ~GaussianComponent() = default;

      using ErrorMatType = MatrixStatic<Dim, Dynamic>;

      void setParamsStdDev(const MatrixStatic<Dim, 1> &StdDev,
                           const MatrixStatic<Dim, 1> &Mean,
                           const MatrixStatic<1, 1> &Weight)
      {
        Mean_ = Mean;
        Weight_ = Weight;

        /** write information matrix directly */
        SqrtInformation_ = StdDev.cwiseInverse().asDiagonal();

        updateScaling_();
      }

      void setParamsCovariance(const MatrixStatic<Dim, Dim> &Covariance,
                               const MatrixStatic<Dim, 1> &Mean,
                               const MatrixStatic<1, 1> &Weight)
      {
        Mean_ = Mean;
        Weight_ = Weight;

        /** use decomposition to get square root information matrix */
        SqrtInformation_ = InverseSquareRoot(Covariance);

        updateScaling_();
      }

      void setParamsInformation(const MatrixStatic<Dim, Dim> &Information,
                                const MatrixStatic<Dim, 1> &Mean,
                                const MatrixStatic<1, 1> &Weight)
      {
        Mean_ = Mean;
        Weight_ = Weight;
        SqrtInformation_ = SquareRoot(Information);

        updateScaling_();
      }

      void setParamsSqrtInformation(const MatrixStatic<Dim, Dim> &SqrtInformation,
                                    const MatrixStatic<Dim, 1> &Mean,
                                    const MatrixStatic<1, 1> &Weight)
      {
        Mean_ = Mean;
        Weight_ = Weight;
        SqrtInformation_ = SqrtInformation;

        updateScaling_();
      }

      void setSqrtInformation(const MatrixStatic<Dim, Dim> &SqrtInformation)
      {
        SqrtInformation_ = SqrtInformation;

        updateScaling_();
      }

      void updateCovariance(const MatrixStatic<Dim, Dim> &Covariance)
      {
        SqrtInformation_ = InverseSquareRoot(Covariance);

        updateScaling_();
      }

      [[nodiscard]] MatrixStatic<Dim, 1> getMean() const
      {
        return Mean_;
      }

      [[nodiscard]] MatrixStatic<1, 1> getWeight() const
      {
        return Weight_;
      }

      [[nodiscard]] MatrixStatic<Dim, Dim> getSqrtInformation() const
      {
        return SqrtInformation_;
      }

      [[nodiscard]] MatrixStatic<Dim, Dim> getCovariance() const
      {
        return (SqrtInformation_.transpose() * SqrtInformation_).inverse();
      }

      void setMean(MatrixStatic<Dim, 1> Mean)
      {
        Mean_ = Mean;
      }

      /** return the part inside the exp() function */
      template <typename T>
      MatrixT<T, Dim, 1 > getExponentialPart(const VectorT<T, Dim> &Error) const
      {
        VectorT<T, Dim> WeightedError;

        /** shift by mean */
        WeightedError = Error + Mean_.template cast<T>();

        /** scale with information matrix */
        WeightedError = SqrtInformation_.template cast<T>() * WeightedError;

        return WeightedError;
      }

      /** return the part before the exp() function */
      template <typename T>
      double getLinearPart(const VectorT<T, Dim> &Error) const
      {
        return Scaling_;
      }

      /** return the maximum value at the mode of the probability function*/
      [[nodiscard]] double getMaximum() const
      {
        return Scaling_;
      }

      /** compute probability for E step of EM */
      Vector computeLikelihood (const ErrorMatType &Errors) const
      {
        /** apply mean */
        ErrorMatType WeightedError = (Errors.array().colwise() + Mean_.array()).matrix();

        /** multiply each row with square root information */
        for (Index n = 0; n < WeightedError.cols(); ++n)
        {
          WeightedError.col(n) = (WeightedError.col(n).transpose() * SqrtInformation_).transpose();
        }

        return ((WeightedError.array().square().colwise().sum() / -2.0).exp() * Scaling_).matrix();
      }

      /** estimate parameters for M step of EM */
      void estimateParameters (const ErrorMatType &Errors,
                               const Vector &Likelihoods,
                               bool  EstimateMean)
      {
        const double LikelihoodSum = Likelihoods.sum();

        /** estimate weight */
        Weight_(0) = LikelihoodSum / static_cast<double>(Likelihoods.rows());

        /** estimate mean */
        if (EstimateMean)
        {
           Mean_ = -((Errors.array().rowwise() * Likelihoods.transpose().array()).rowwise().sum() / LikelihoodSum).matrix();
        }

        /** estimate covariance */
        MatrixStatic<Dim,Dim> Covariance = MatrixStatic<Dim,Dim>::Zero();
        for (Index n = 0; n < Errors.cols(); ++n)
        {
          MatrixStatic<Dim, 1> Diff = Errors.col(n) + Mean_;
          Covariance += Diff * Diff.transpose() * Likelihoods(n);
        }
        Covariance.array() /= LikelihoodSum;
        SqrtInformation_ = InverseSquareRoot(Covariance);

        /** check for degenerated square root information matrix */
        if(SqrtInformation_.array().isFinite().all() == false)
        {
          /** disable component if degenerated */
          Weight_.setZero();
          Mean_.setZero();
          SqrtInformation_ = MatrixStatic<Dim, Dim>::Identity();
        }

        /** check for extreme numerical values */
        if((SqrtInformation_.array() > 1e10).any() == true)
        {
          #ifndef NDEBUG
          PRINT_WARNING("Square root information is too high: ", SqrtInformation_.maxCoeff());
          PRINT_WARNING("Limit Square root information to 1e10 for numerical reasons!");
          #endif
          SqrtInformation_ = (SqrtInformation_.array() < 1e10).select(SqrtInformation_, 1e10);
        }

        updateScaling_();
      }

      void estimateParametersMAP (const ErrorMatType &Errors,
                                  const Vector &Likelihoods,
                                  const double DirichletConcentration,
                                  const double DirichletConcentrationSum,
                                  const double NormalInfoScaling,
                                  const VectorStatic<Dim> &NormalMean,
                                  const MatrixStatic<Dim,Dim> &WishartScatterMatrix,
                                  const double WishartDOF,
                                  bool EstimateMean)
      {
        /** Implementation based on:
        * Kevin Murphy
        * Machine learning: a probabilistic perspective, Section 11.4.2
        * 2012
        */

        /** pre-calculate shared values */
        const double LikelihoodSum = Likelihoods.sum();

        /** estimate weight */
        Weight_(0) = std::max((LikelihoodSum + DirichletConcentration - 1.0)
                     /
                     (DirichletConcentrationSum + static_cast<double>(Likelihoods.rows())), 0.0);

        /** estimate mean */
        const VectorStatic<Dim> MeanML = (Errors.array().rowwise() * Likelihoods.transpose().array()).rowwise().sum() / LikelihoodSum;
        if (EstimateMean)
        {
          Mean_ = -(MeanML*LikelihoodSum + NormalMean*NormalInfoScaling) / (NormalInfoScaling + LikelihoodSum);
        }

        /** estimate covariance */
        MatrixStatic<Dim,Dim> ScatterML = MatrixStatic<Dim,Dim>::Zero();
        for (Index n = 0; n < Errors.cols(); ++n)
        {
          const MatrixStatic<Dim, 1> Diff = Errors.col(n) - MeanML;
          ScatterML += Diff * Diff.transpose() * Likelihoods(n);
        }

        const MatrixStatic<Dim,Dim> DenominatorInfo = WishartScatterMatrix
                                                    + ScatterML
                                                    + NormalInfoScaling*LikelihoodSum/(NormalInfoScaling + LikelihoodSum)
                                                      *(MeanML - NormalMean)*(MeanML - NormalMean).transpose();

        const double EnumeratorInfo = WishartDOF + LikelihoodSum + Dim + 2;

        SqrtInformation_ = SquareRoot<Dim,double>(EnumeratorInfo * Inverse<Dim,double>(DenominatorInfo));

        /** check for degenerated square root information matrix */
        if(SqrtInformation_.array().isFinite().all() == false)
        {
          /** disable component if degenerated */
          Weight_.setZero();
          Mean_.setZero();
          SqrtInformation_ = MatrixStatic<Dim, Dim>::Identity();
        }

        updateScaling_();
      }

      /** compute negative log-likelihood of a sample vector */
      Vector computeNegLogLikelihood (const ErrorMatType &Errors, const bool EvalAsError = true) const
      {
        /** apply mean (sign depends on the interpretation of a sample) */
        ErrorMatType WeightedError;
        if (EvalAsError)
        {
          WeightedError = (Errors.array().colwise() + Mean_.array()).matrix();
        }
        else
        {
          WeightedError = (Errors.array().colwise() - Mean_.array()).matrix();
        }

        /** multiply each row with square root information */
        for (int n = 0; n < WeightedError.cols(); ++n)
        {
          WeightedError.col(n) = (WeightedError.col(n).transpose() * SqrtInformation_).transpose();
        }

        return ((WeightedError.array().square().colwise().sum() / 2.0) - log(Scaling_)).matrix();
      }

      [[nodiscard]] bool checkParameters () const
      {
        bool Passed = true;

        /** check weight */
        if(!Weight_.allFinite())
        {
          PRINT_WARNING("Weight is not finite!");
          Passed = false;
        }
        else if(Weight_(0) <= 0)
        {
          PRINT_WARNING("Weight is not positive!");
          Passed = false;
        }
        else if(Weight_(0) > 1)
        {
          PRINT_WARNING("Weight is bigger than one!");
          Passed = false;
        }

        /** check mean */
        if(Mean_.allFinite() == false)
        {
          PRINT_WARNING("Mean is not finite!");
          Passed = false;
        }

        /** check uncertainty */
        if(SqrtInformation_.allFinite() == false)
        {
          PRINT_WARNING("Square root information is not finite!");
          Passed = false;
        }
        else
        {
          if(IsPositiveSemidefinite<Dim>(SqrtInformation_) == false)
          {
            PRINT_WARNING("Square root information is not positive semi-definite!");
            Passed = false;
          }
        }

        /** check scaling */
        if(!std::isfinite(Scaling_))
        {
          PRINT_WARNING("Scaling is not finite!");
          Passed = false;
        }

        return Passed;
      }

      /** draw a sample */
      VectorVectorSTL<Dim> DrawSamples(const int Number) const
      {
        /** prepare sample transformation */
        Eigen::SelfAdjointEigenSolver<MatrixStatic<Dim,Dim>> Solver(this->getCovariance());
        const MatrixStatic<Dim,Dim> Transform = Solver.eigenvectors() * Solver.eigenvalues().cwiseSqrt().asDiagonal();

        /** set up Gaussian generator */
        std::default_random_engine Generator;
        std::normal_distribution<double> Gaussian(0, 1);

        /** crate storage */
        VectorVectorSTL<Dim> SampleVector;

        /** create one [Dim x 1] sample per iteration */
        for (int n = 0; n < Number; n++)
        {
          libRSF::VectorStatic<Dim> Sample;

          /** draw samples */
          for (int m = 0; m < Dim; m++)
          {
            Sample(m) = Gaussian(Generator);
          }

          /** transform according to distribution */
          Sample = Transform * Sample + this->Mean_;

          /** store them */
          SampleVector.push_back(Sample);
        }

        return SampleVector;
      }

    private:
      void updateScaling_()
      {
        Scaling_ = calculateNormalization_();
      }

      double calculateNormalization_()
      {
        return Weight_(0, 0) * SqrtInformation_.determinant();
      }

      MatrixStatic<Dim, 1> Mean_;
      MatrixStatic<1, 1> Weight_;
      MatrixStatic<Dim, Dim> SqrtInformation_;
      double Scaling_;
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

  /** merge two Gaussians */
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
