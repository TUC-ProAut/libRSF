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

#include "error_models/GaussianMixture.h"

namespace libRSF
{
  template<>
  void GaussianMixture<1>::addDiagonal(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight)
  {
    GaussianComponent<1> Gaussian;

    if(StdDev.size() == Mean.size() && StdDev.size() == Weight.size())
    {
      for(size_t nDim = 0; nDim < StdDev.size(); ++nDim)
      {
        Gaussian.setParamsStdDev(StdDev.segment<1>(nDim), Mean.segment<1>(nDim), Weight.segment<1>(nDim));
        addComponent(Gaussian);
      }
    }
    else
    {
      std::cerr << "Error in GaussianMixture::addDiagonal(): Parameter dimension isn't equal! "
                <<"StdDev: "  << StdDev.size()  << " "
                <<"Mean: "    << Mean.size()    << " "
                <<"Weight: "  << Weight.size()  << " " << std::endl;

    }

  }

  template<>
  GaussianMixture<1>::GaussianMixture(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight)
  {
    addDiagonal(Mean, StdDev, Weight);
  }

  template<>
  Eigen::VectorXd GaussianMixture<1>::removeOffset()
  {
    size_t NumberOfComponents = this->getNumberOfComponents();
    Eigen::VectorXd MeanLOS;

    this->sortComponentsByWeight();
    double MinimumWeight = std::min(_Mixture.at(0).getWeight()(0), 0.2);

    /** remove offset of the first "LOS" component */
    this->sortComponentsByMean();
    for(int i = 0; i < NumberOfComponents; ++i)
    {
      if(_Mixture.at(i).getWeight()(0) >= MinimumWeight)
      {
        MeanLOS = _Mixture.at(i).getMean();
        break;
      }
    }

    for(int i = 0; i < NumberOfComponents; ++i)
    {
      _Mixture.at(i).setMean(_Mixture.at(i).getMean() - MeanLOS);
    }
    return MeanLOS;
  }

  template<>
  void GaussianMixture<1>::removeGivenOffset(Eigen::VectorXd Offset)
  {
    size_t NumberOfComponents = this->getNumberOfComponents();

    /** remove offset of the given component */
    for(int i = 0; i < NumberOfComponents; ++i)
    {
      _Mixture.at(i).setMean(_Mixture.at(i).getMean() - Offset);
    }
  }

  /** variational bayesian inference */
  template <>
  void GaussianMixture<1>::estimateWithVBI(std::vector<double> &Data, double Nu)
  {
    /** only 1D problems !!! */
    #define Dimension 1

    /** set up problem */
    size_t N = Data.size();
    size_t M = _Mixture.size();

    /** N x M matrix */
    Eigen::MatrixXd Likelihood;
    Likelihood.resize(N, M);

    /** map data to eigen vector */
    ErrorMatType DataVector = Eigen::Map<const ErrorMatType, Eigen::Unaligned>(Data.data(), N, Dimension);
    /** change sign to match own GMM definition */
    DataVector.array() *= -1;

    double DataMean = DataVector.array().mean();
    double DataCov = (DataVector.array() - DataMean).square().sum()/(DataVector.size()-1);

    /** convergence criterion */
    double const LikelihoodTolerance = 1e-6;
    int const MaxIterations = 1000;

    /** pruning criterion */
    double MinWeight = 1.0/N;

    /** define priors */
    double const Beta = 1e-6;                 /**< prior over mean */
    double const V = DataCov/Nu;              /**< Wishart prior for I */

    /** initialize  variables */
    std::vector<double> NuInfo;
    std::vector<double> VInfo;
    std::vector<double> InfoMean;
    std::vector<double> MeanMean;
    std::vector<double> Weights;

    /** initialize estimated parameters */
    for(size_t m = 0; m < M; ++m)
    {
      NuInfo.push_back(Nu);
      VInfo.push_back(V);
      InfoMean.push_back(0.0); /**< will be overwritten */
      MeanMean.push_back(0.0); /**< will be overwritten */
      Weights.push_back(_Mixture.at(m).getWeight()(0));
    }

    /** convergence criteria (not variational!)*/
    double LikelihoodSumOld = 1e40;
    double LikelihoodSumNew;

    /** first likelihood estimation is not variational! */
    LikelihoodSumNew = computeLikelihood(DataVector, Likelihood);

    /** repeat until convergence or 100 iterations*/
    int i;
    for(i = 0; i < MaxIterations; ++i)
    {
      bool ReachedConvergence = false;
      bool PerfomedRemove = false;

      /** Expectation step */

      /** pre-calculate some multi-use variables */
      Eigen::VectorXd SumLike = Likelihood.colwise().sum();
      Eigen::VectorXd SumLikeX = (Likelihood.array().colwise() * DataVector.array()).colwise().sum();
      Eigen::VectorXd SumLikeXX = (Likelihood.array().colwise() * DataVector.array().square()).colwise().sum();

      double MuMuEx, InfoEx, LogInfoEx;
      /** iterate over GMM components */
      for(size_t m = 0; m < M; ++m)
      {

        /** calculate mean */
        InfoMean.at(m) = Beta + NuInfo.at(m) / VInfo.at(m) * SumLike(m);
        MeanMean.at(m) = NuInfo.at(m) / VInfo.at(m) / InfoMean.at(m) * SumLikeX(m);

        /** calculate variance */
        NuInfo.at(m) = Nu + SumLike(m);
        MuMuEx = 1.0/InfoMean.at(m) + MeanMean.at(m)*MeanMean.at(m);

        VInfo.at(m) = V + SumLikeXX(m) - 2*MeanMean.at(m)*SumLikeX(m) + MuMuEx * SumLike(m);

        /** variational likelihood */
        InfoEx = NuInfo.at(m)/VInfo.at(m);
        LogInfoEx = Eigen::numext::digamma(NuInfo.at(m)/2.0) + log(2.0) - log(VInfo.at(m));

        Likelihood.col(m) = (LogInfoEx/2.0
                             + log(Weights.at(m))
                             - 0.5*InfoEx*
                             (
                               DataVector.array().square()
                               - 2.0 * MeanMean.at(m) * DataVector.array()
                               + MuMuEx
                             )
                            ).array().exp();
      }

      /** "Maximization step" */

      /** remove NaNs */
      Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);

      /** calculate relative likelihood  */
      LikelihoodSumNew = Likelihood.sum();
      Eigen::VectorXd LikelihoodRowSum = Likelihood.rowwise().sum();

      for(size_t m = 0; m < M; ++m)
      {
        Likelihood.col(m).array() /= LikelihoodRowSum.array();
      }

      /** remove NaNs (occur if sum of likelihoods is zero) */
      Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 1.0/M);

      /** calculate weights */
      for(size_t m = 0; m < M; ++m)
      {
        Weights.at(m)= Likelihood.col(m).sum() / Likelihood.rows();
      }

      /** remove useless components */
      for(int m = M-1; m >=0 ; --m)
      {
        if(Weights.at(m) < MinWeight)
        {
          NuInfo.erase(NuInfo.begin() + m);
          VInfo.erase(VInfo.begin() + m);
          InfoMean.erase(InfoMean.begin() + m);
          MeanMean.erase(MeanMean.begin() + m);
          Weights.erase(Weights.begin() + m);
          RemoveColumn(Likelihood,m);
          _Mixture.erase(_Mixture.begin() + m);

          M -= 1;
          /** enforce an additional iteration after removal and reset likelihood */
          PerfomedRemove = true;
          LikelihoodSumNew = 1e40;
        }
      }

      /** check for convergence */
      double LikelihoodChange = std::abs(LikelihoodSumOld - LikelihoodSumNew)/LikelihoodSumNew;

      if(LikelihoodChange < LikelihoodTolerance)
      {
        ReachedConvergence = true;
      }
      else
      {
        ReachedConvergence = false;
      }

      /** terminate loop */
      if(ReachedConvergence && !PerfomedRemove)
      {
        break;
      }
      else
      {
        LikelihoodSumOld = LikelihoodSumNew;
      }

    }

    /** update mixtures */
    for(size_t m = 0; m < M; ++m)
    {
      Eigen::Matrix<double,1,1> Mean;
      Eigen::Matrix<double,1,1> Info;
      Eigen::Matrix<double,1,1> Weight;

      Mean << MeanMean.at(m);
      Info << NuInfo.at(m)/VInfo.at(m);
      Weight << Weights.at(m);

      _Mixture.at(m).setParamsInformation(Info, Mean, Weight);
    }

  }
}
