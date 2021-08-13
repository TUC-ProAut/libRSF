/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
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
 * @file GaussianMixture.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A weighted mixture of multiple Gaussians. Not intended for standalone usage.
 * @copyright GNU Public License.
 *
 */

#ifndef GAUSSIANMIXTURE_H
#define GAUSSIANMIXTURE_H

#include "GaussianComponent.h"
#include "../Data.h"
#include "../Misc.h"
#include "../Messages.h"
#include "../Statistics.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/SpecialFunctions>

#include <algorithm>

namespace libRSF
{
  template <int Dim>
  class GaussianMixture
  {
    public:
      GaussianMixture() = default;
      virtual ~GaussianMixture() = default;

      typedef MatrixT<double, Dim, Dynamic> ErrorMatType;

      /** config for mixture estimation */
      struct EstimationConfig
      {
        typedef ErrorModelTuningType EstimationAlgorithmType;
        /** estimation algorithm */
        EstimationAlgorithmType EstimationAlgorithm = EstimationAlgorithmType::EM;

        /** mixture properties */
        bool EstimateMean = true;

        /** termination criteria */
        int MaxIterations = 100;
        double MinLikelihoodChange = 1e-5;

        /** refuse estimation with less samples*/
        double MinimalSamples = 15;

        /** hyper-priors */
        double PriorDirichletConcentration = 1;

        double PriorNormalInfoScaling = 1e-6;
        VectorStatic<Dim> PriorNormalMean = VectorStatic<Dim>::Zero();

        double PriorWishartDOF = Dim - 1 + 2;
        MatrixStatic<Dim, Dim> PriorWishartScale = MatrixStatic<Dim, Dim>::Identity();

        /** remove components if the number of assigned samples is too low */
        bool RemoveSmallComponents = false;
        /** the minimal number of samples that have to support a component*/
        double MinSamplePerComponent = Dim + 1;

        /** check the Bhattacharyya distance between all components to merge similar ones */
        bool MergeSimiliarComponents = false;
        double MergingThreshhold = 0.1;
      };

      /** Bayesian representation of the parameter estimation problem */
      struct BayesianState
      {
        VectorVectorSTL<1> Weights;

        VectorVectorSTL<Dim> MeanMean;
        MatrixVectorSTL<Dim, Dim> InfoMean;

        VectorVectorSTL<1> NuInfo;
        MatrixVectorSTL<Dim, Dim> VInfo;
      };

      bool estimate(const std::vector<double> &Data, const EstimationConfig &Config)
      {
        const int N = Data.size() / Dim;

        /** check size */
        if (N < Config.MinimalSamples)
        {
          PRINT_WARNING("Sample Size to low: ", N);
          return false;
        }

        /** map data to eigen vector */
        const ErrorMatType DataMatrix = Eigen::Map<const ErrorMatType, Eigen::Unaligned, Eigen::Stride<1, Dim>>(Data.data(), Dim, N);

        return this->estimate(DataMatrix, Config);
      }

      bool estimate(const MatrixStatic<Dim, Dynamic> &DataMatrix, const EstimationConfig &Config)
      {
        const int N = DataMatrix.cols();

        /** check size */
        if (N < Config.MinimalSamples)
        {
          PRINT_WARNING("Sample Size to low: ", N);
          return false;
        }

        /** estimate a good covariance prior*/
        EstimationConfig ModifiedConfig = Config;
        if (Config.EstimationAlgorithm == ErrorModelTuningType::EM_MAP || Config.EstimationAlgorithm == ErrorModelTuningType::VBI)
        {
          ModifiedConfig.PriorWishartScale = EstimateSampleCovariance(DataMatrix) / Config.PriorWishartDOF;
        }

        /** init*/
        static BayesianState VBIState;
        bool ReachedMaxIteration = false;
        bool Converged = false;
        bool Merged = false;
        bool Prunned = false;
        double LikelihoodSumOld = 0;
        double LikelihoodSum = 0;
        int k = 1;

        /** iterate until convergence */
        while ((Converged == false && ReachedMaxIteration == false) || Merged == true || Prunned == true)
        {
          /** init required varaibles */
          int M = _Mixture.size();

          switch (Config.EstimationAlgorithm)
          {
            case ErrorModelTuningType::EM:
              {
                /** E-step */
                Matrix Probability(M, N);
                LikelihoodSum = this->computeProbability(DataMatrix, Probability);

                /** M-Step maximum likelihood */
                this->computeMixtureParameters(DataMatrix, Probability, ModifiedConfig);
              }
              break;

            case ErrorModelTuningType::EM_MAP:
              {
                /** E-step */
                Matrix Probability(M, N);
                LikelihoodSum = this->computeProbability(DataMatrix, Probability);

                /** M-Step maximum-a-posteriori*/
                this->computeMixtureParametersMAP(DataMatrix, Probability, ModifiedConfig);
              }
              break;

            case ErrorModelTuningType::VBI:
              {
                /** multivariate VBI*/
                static Matrix Probability(M, N);
                if (k == 1)
                {
                  /** first likelihood is not variational */
                  this->computeProbability(DataMatrix, Probability);

                  /** reset state */
                  VBIState = BayesianState();
                }
                LikelihoodSum = this->doVariationalStep(DataMatrix, Probability, VBIState, ModifiedConfig);
              }
              break;

            default:
              PRINT_ERROR("Wrong mixture estimation algorithm type!");
              break;
          }

          /** post-process mixture */
          if (Config.RemoveSmallComponents &&
              (Config.EstimationAlgorithm == ErrorModelTuningType::EM ||
               Config.EstimationAlgorithm == ErrorModelTuningType::EM_MAP))
          {
            Prunned = this->prunMixture(Config.MinSamplePerComponent / N);
          }
          else
          {
            Prunned = this->prunMixture(0.0);
          }

          if (Config.MergeSimiliarComponents &&
              (Config.EstimationAlgorithm == ErrorModelTuningType::EM ||
               Config.EstimationAlgorithm == ErrorModelTuningType::EM_MAP))
          {
            Merged = this->reduceMixture(Config.MergingThreshhold);
          }
          else
          {
            Merged = false;
          }

          /** check for convergence */
          double LikelihoodChange = std::abs(LikelihoodSum - LikelihoodSumOld) / LikelihoodSum;
          if (k > 1 && LikelihoodChange < Config.MinLikelihoodChange)
          {
            Converged = true;
          }
          else
          {
            Converged = false;
          }

          /** check for iterations */
          if (k >= Config.MaxIterations)
          {
            ReachedMaxIteration = true;
          }
          else
          {
            ReachedMaxIteration = false;
          }

          /** save for next iteration */
          LikelihoodSumOld = LikelihoodSum;

          /** increment iteration counter */
          k++;
        }

        if (Config.EstimationAlgorithm == ErrorModelTuningType::VBI)
        {
          this->extractParameterFromBayes(VBIState);
        }

        /** check if any GMM parameter is consistent after the estimation (only in debug mode)*/
        bool GMMConsistency = true;
#ifndef NDEBUG
        for (int m = 0; m < static_cast<int>(_Mixture.size()); m++)
        {
          if (_Mixture.at(m).checkParameters() == false)
          {
            PRINT_ERROR("Check of GMM component ", m+1, " of ", _Mixture.size(), " failed!");
            GMMConsistency = false;
          }
        }

        PRINT_LOGGING("GMM estimation started with ", N, " samples.");
        PRINT_LOGGING("GMM estimation ended with ", _Mixture.size(), " components, after ", k - 1, " iterations.");
        this->printParameter();
#endif

        return GMMConsistency;
      }

      /** init with increasing uncertainties */
      void initSpread(const int Components, const double BaseStdDev, const bool IsAssymetric = false)
      {
        libRSF::GaussianComponent<Dim> Component;

        MatrixStatic<Dim, 1> Mean = MatrixStatic<Dim, 1>::Zero();
        const MatrixStatic<1, 1> Weight = MatrixStatic<1, 1>::Ones() / Components;
        const MatrixStatic<Dim, Dim> SqrtInfo = MatrixStatic<Dim, Dim>::Identity() * (1.0 / BaseStdDev);

        this->clear();
        for (int nComponent = 0; nComponent < static_cast<int>(Components); ++nComponent)
        {
          if(nComponent > 0 && IsAssymetric)
          {
            Mean = MatrixStatic<Dim, 1>::Ones() * BaseStdDev*BaseStdDev * std::pow(10, nComponent-1);
          }
          Component.setParamsSqrtInformation(SqrtInfo * std::pow(0.1, nComponent), Mean, Weight);
          this->addComponent(Component);
        }
      }

      /** only for 1D GMMs */
      GaussianMixture(Vector Mean, Vector StdDev, Vector Weight);
      void addDiagonal(Vector Mean, Vector StdDev, Vector Weight);

      void addComponent(GaussianComponent<Dim> Gaussian)
      {
        _Mixture.emplace_back(Gaussian);
      }

      /** for distribution-to-distribution models */
      void inflateWithCov(MatrixStatic<Dim, Dim> Cov)
      {
        for (int i = 0; i < static_cast<int>(_Mixture.size()); ++i)
        {
          MatrixStatic<Dim, Dim> CovNew;

          CovNew = Cov + _Mixture.at(i).getCovariance();

          _Mixture.at(i).updateCovariance(CovNew);
        }
      }

      void removeLastComponent()
      {
        _Mixture.pop_back();
      }

      void clear()
      {
        _Mixture.clear();
      }

      int getNumberOfComponents() const
      {
        return _Mixture.size();
      }

      void getMixture(std::vector<GaussianComponent<Dim>>& Mixture)
      {
        Mixture = _Mixture;
      }

      void computeLikelihood(const ErrorMatType &DataVector, Matrix &Likelihood) const
      {
        const int M = _Mixture.size(); /**< number of components */
        const int N = DataVector.cols(); /**< number of data samples */

        /** adapt size of output matrix */
        Likelihood.resize(M, N);

        /** loop over components*/
        for (int m = 0; m < M; ++m)
        {
          Likelihood.row(m) = _Mixture.at(m).computeLikelihood(DataVector);
        }

        /** remove NaNs */
        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);
      }

      double computeProbability(const ErrorMatType &DataVector, Matrix &Probability) const
      {
        const int M = _Mixture.size(); /**< number of components */
        const int N = DataVector.cols(); /**< number of data samples */

        /** get likelihood */
        this->computeLikelihood(DataVector, Probability);
        const double LikelihoodSum = Probability.sum();

        /** normalize over all components */
        for (int n = 0; n < N; ++n)
        {
          Probability.col(n) /= Probability.col(n).sum();
        }

        /** remove NaNs (occur if sum of likelihoods is zero) */
        Probability = (Probability.array().isFinite()).select(Probability, 1.0 / M);

        return LikelihoodSum;
      }

      int computeMostLikelyComponent(MatrixStatic<Dim, 1> &Error) const
      {
        /** compute all likelihoods */
        Matrix Likelihood;
        this->computeLikelihood(Error, Likelihood);

        /** find maximum */
        int MaxIndexCol, MaxIndexRow;

        Likelihood.maxCoeff(&MaxIndexRow, &MaxIndexCol);

        return MaxIndexRow;
      }

      double computeNegLogLikelihood(const ErrorMatType &DataVector, Matrix &NegLogLikelihood, const bool EvalAsError = true) const
      {
        const int M = _Mixture.size(); /**< number of components */
        const int N = DataVector.cols(); /**< number of data samples */

        /** adapt size of output matrix */
        NegLogLikelihood.resize(M, N);

        /** loop over components*/
        for (int m = 0; m < M; ++m)
        {
          NegLogLikelihood.row(m) = _Mixture.at(m).computeNegLogLikelihood(DataVector, EvalAsError);
        }

        /** remove NaNs */
        NegLogLikelihood = (NegLogLikelihood.array().isFinite()).select(NegLogLikelihood, 1e100);

        return NegLogLikelihood.sum();
      }

      void extractParameterFromBayes(const BayesianState &State)
      {
        /** delete old mixture */
        _Mixture.clear();

        /** extract parameter component-wise */
        const int GMMSize = State.Weights.size();
        for (int m = 0; m < GMMSize; ++m)
        {
          libRSF::GaussianComponent<Dim> Component;
          Component.setParamsInformation(State.NuInfo.at(m)(0)*State.VInfo.at(m).inverse(), -State.MeanMean.at(m), State.Weights.at(m));
          this->addComponent(Component);
        }
      }

      double doVariationalStep(const ErrorMatType &DataMatrix,
                               Matrix &Probability,
                               BayesianState &VBIState,
                               const EstimationConfig &Config) const
      {

        /** Implementation of:
         * Corduneanu, A. & Bishop, C.
         * Variational Bayesian Model Selection for Mixture Distributions
         * Proc. of Intl. Conf. on Artificial Intelligence and Statistics (AISTATS)
         * 2001
         */

        /** set hyperpriors */
        const MatrixStatic<Dim,Dim> V = Config.PriorWishartScale;
        const MatrixStatic<Dim, Dim> Beta = Config.PriorNormalInfoScaling * MatrixStatic<Dim,Dim>::Identity();
        const double Nu = Config.PriorWishartDOF;

        /** adapt to current size */
        const int GMMSize = VBIState.Weights.size();
        const int SampleSize = DataMatrix.cols();
        if (GMMSize < Probability.rows())
        {
          for (int n = GMMSize; n < Probability.rows(); ++n)
          {
            VBIState.Weights.push_back(this->_Mixture.at(n).getWeight());

            VBIState.InfoMean.push_back(Beta); /**< will be overwritten */
            VBIState.MeanMean.push_back(Config.PriorNormalMean); /**< will be overwritten */

            VBIState.NuInfo.push_back((Vector1() << Nu).finished());
            VBIState.VInfo.push_back(V);
          }
        }

        /** Expectation step */

        /** pre-calculate some multi-use variables */
        Vector SumLike = Probability.rowwise().sum();
        MatrixStatic<Dynamic, Dim> SumLikeX = Probability * DataMatrix.transpose();

        MatrixVectorSTL<Dim, Dim> XX;
        for (int n = 0; n < SampleSize; n++)
        {
          XX.push_back(DataMatrix.col(n) * DataMatrix.col(n).transpose());
        }

        MatrixVectorSTL<Dim, Dim> SumLikeXX;
        for (int m = 0; m < GMMSize; m++)
        {
          MatrixStatic<Dim,Dim> SumLikeXXComp = MatrixStatic<Dim,Dim>::Zero();
          for (int n = 0; n < SampleSize; n++)
          {
            SumLikeXXComp += XX.at(n) * Probability(m,n);
          }
          SumLikeXX.push_back(SumLikeXXComp);
        }

        /** iterate over GMM components */
        for (int m = 0; m < GMMSize; ++m)
        {
          /** <T_i> */
          MatrixStatic<Dim,Dim> InfoEx = VBIState.NuInfo.at(m)(0) * Inverse(VBIState.VInfo.at(m));

          /** update mean posterior */
          VBIState.InfoMean.at(m) = Beta + InfoEx * SumLike(m);

          if (Config.EstimateMean == true)
          {
            VBIState.MeanMean.at(m) = Inverse(VBIState.InfoMean.at(m)) * InfoEx * SumLikeX.row(m).transpose();
          }

          /** update variance posterior */
          VBIState.NuInfo.at(m)(0) = Nu + SumLike(m);

          /** <Mu_i*Mu_i^T> */
          const MatrixStatic<Dim,Dim> MuMuEx = Inverse(VBIState.InfoMean.at(m)) + VBIState.MeanMean.at(m) * VBIState.MeanMean.at(m).transpose();

          MatrixStatic<Dim,Dim> SumXPMu = MatrixStatic<Dim,Dim>::Zero();
          for (int n = 0; n < SampleSize; n++)
          {
            SumXPMu += DataMatrix.col(n) * Probability(m,n) * VBIState.MeanMean.at(m).transpose();
          }

          VBIState.VInfo.at(m) = V
                               + SumLikeXX.at(m)
                               - SumXPMu
                               - VBIState.MeanMean.at(m) * SumLikeX.row(m)
                               + MuMuEx * SumLike(m);

          /** variational likelihood */
          /** <ln|T_i|> */
          double LogInfoEx = 0;
          for (int d = 1; d <= Dim; d++)
          {
            LogInfoEx += Eigen::numext::digamma(0.5 * (VBIState.NuInfo.at(m)(0) + 1 - d));
          }
          LogInfoEx += Dim * log(2.0) - log(VBIState.VInfo.at(m).determinant());

          /** update <T_i> */
          InfoEx = VBIState.NuInfo.at(m)(0) * Inverse(VBIState.VInfo.at(m));

          /** <Mu_i> */
          const VectorStatic<Dim> MeanEx = VBIState.MeanMean.at(m);

          Probability.row(m).fill(0.5 * LogInfoEx + log(VBIState.Weights.at(m)(0)));
          for (int n = 0; n < SampleSize; n++)
          {
            Probability(m,n) -= 0.5 * ( InfoEx * (XX.at(n)
                                      - MeanEx * DataMatrix.col(n).transpose()
                                      - DataMatrix.col(n) * MeanEx.transpose()
                                      + MuMuEx) ).trace();
          }
          Probability.row(m) = Probability.row(m).array().exp();
        }

        /** "Maximization step" --> update probability */
        /** remove NaN */
        Probability = (Probability.array().isFinite()).select(Probability, 0.0);

        /** calculate relative likelihood  */
        double LikelihoodSum = Probability.sum();
        for (int n = 0; n < static_cast<int>(Probability.cols()); ++n)
        {
          Probability.col(n) /= Probability.col(n).sum();
        }

        /** remove NaN again (occur if sum of likelihoods is zero) */
        Probability = (Probability.array().isFinite()).select(Probability, 1.0 / GMMSize);

        /** calculate weights */
        for (int m = 0; m < GMMSize; ++m)
        {
          VBIState.Weights.at(m)(0) = Probability.row(m).sum() / Probability.cols();
        }

        /** remove useless or degenerated components */
        for (int m = GMMSize-1; m >=0 ; --m)
        {
          if(!(VBIState.Weights.at(m)(0) >= (Config.MinSamplePerComponent / SampleSize / 2.0)) && VBIState.Weights.size() > 1)/**< catches NaN also */
          {
            /** remove component from posterior states */
            VBIState.NuInfo.erase(VBIState.NuInfo.begin() + m);
            VBIState.VInfo.erase(VBIState.VInfo.begin() + m);
            VBIState.InfoMean.erase(VBIState.InfoMean.begin() + m);
            VBIState.MeanMean.erase(VBIState.MeanMean.begin() + m);
            VBIState.Weights.erase(VBIState.Weights.begin() + m);

            /** remove row from probability matrix */
            Matrix ProbTrans = Probability.transpose();
            RemoveColumn(ProbTrans,m);
            Probability = ProbTrans.transpose();

            /** enforce an additional iteration after removal by resetting likelihood */
            LikelihoodSum = 1e40;
          }
        }

        return LikelihoodSum;
      }

      void computeMixtureParameters(const ErrorMatType &DataVector, const Matrix &Likelihood, const EstimationConfig &Config)
      {
        const int M = _Mixture.size();
        for (int m = 0; m < M; m++)
        {
          _Mixture.at(m).estimateParameters(DataVector, Likelihood.row(m), Config.EstimateMean);
        }
      }

      void computeMixtureParametersMAP(const ErrorMatType &DataVector, const Matrix &Likelihood, const EstimationConfig &Config)
      {
        const int M = _Mixture.size();

        const double DirichletSum = (Config.PriorDirichletConcentration - 1) * M;

        for (int m = 0; m < M; m++)
        {
          _Mixture.at(m).estimateParametersMAP(DataVector,
                                               Likelihood.row(m),
                                               Config.PriorDirichletConcentration,
                                               DirichletSum,
                                               Config.PriorNormalInfoScaling,
                                               Config.PriorNormalMean,
                                               Config.PriorWishartScale,
                                               Config.PriorWishartDOF,
                                               Config.EstimateMean);
        }
      }

      /** remove components, that are to small */
      bool prunMixture(const double MinWeight)
      {
        bool Prunned = false;

        for (int m = _Mixture.size() - 1; m >= 0; m--)
        {
          if (_Mixture.at(m).getWeight()(0) < MinWeight || _Mixture.at(m).getWeight()(0) == 0.0)
          {
            _Mixture.erase(_Mixture.begin() + m);
            Prunned = true;
          }
        }

        return Prunned;
      }

      /** merge components that are identical */
      bool reduceMixture(const double BhattacharyyaLimit)
      {
        bool PerformedMerge = false;
        int M = _Mixture.size();

        for (int m1 = 0; m1 < M - 1; ++m1)
        {
          for (int m2 = m1 + 1; m2 < M; ++m2)
          {
            /** calculate metric */
            double d = CalculateBhattacharyyaDistance(_Mixture.at(m1), _Mixture.at(m2));

            /** perform merge if the distance exceed a specific tuning parameter*/
            if (exp(-d) > BhattacharyyaLimit)
            {
              _Mixture.at(m1) = MergeGaussians<Dim>(_Mixture.at(m1), _Mixture.at(m2));
              _Mixture.erase(_Mixture.begin() + m2);

              M -= 1;
              m2 -= 1;

              PerformedMerge = true;
            }
          }
        }
        return PerformedMerge;
      }

      void printParameter() const
      {
        PRINT_LOGGING("GMM Parameter: Mean       StdDev-Diagonal       Weight");
        for (int n = 0; n < static_cast<int>(_Mixture.size()); ++n)
        {
          PRINT_LOGGING("Component ", n + 1, ":   ",
                        _Mixture.at(n).getMean().transpose(), "       ",
                        _Mixture.at(n).getSqrtInformation().diagonal().cwiseInverse().transpose(), "  ",
                        _Mixture.at(n).getWeight());
        }
      }

      /** query error values for a specific component */
      template <typename T>
      VectorT<T, Dim> getExponentialPartOfComponent(int NumberOfComponent, const VectorT<T, Dim> &Error) const
      {
        return _Mixture.at(NumberOfComponent).getExponentialPart(Error);
      }

      template <typename T>
      double getLinearPartOfComponent(int NumberOfComponent, const VectorT<T, Dim> &Error) const
      {
        return _Mixture.at(NumberOfComponent).getLinearPart(Error);
      }

      double getMaximumOfComponent(int NumberOfComponent) const
      {
        return _Mixture.at(NumberOfComponent).getMaximum();
      }

      /** sampling */
      VectorVectorSTL<Dim> DrawSamples(const int Number) const
      {
        /** crate storage */
        VectorVectorSTL<Dim> SampleVector;

        /** loop over components and draw samples */
        const int NumComp = this->getNumberOfComponents();
        for (int n = 0; n < NumComp; n++)
        {
          /** use weight of components for number of samples*/
          int CurrentNumber;
          if (n < NumComp-1)
          {
            CurrentNumber = round(Number * this->_Mixture.at(n).getWeight()(0));
          }
          else
          {
            /** fill with last component to avoid rund-off */
            CurrentNumber = Number - SampleVector.size();
          }

          /** draw */
          const VectorVectorSTL<Dim> CurrentSamples = this->_Mixture.at(n).DrawSamples(CurrentNumber);

          /** append */
          SampleVector.insert(std::end(SampleVector), std::begin(CurrentSamples), std::end(CurrentSamples));
        }

        return SampleVector;
      }

      /** estimate mode of the GMM */
      VectorStatic<Dim> EstimateMode() const
      {
        /** draw samples */
        VectorVectorSTL<Dim> Samples = DrawSamples(1e4*Dim);

        /** evaluate samples */
        VectorStatic<Dim> BestSample = Samples.front();
        Matrix TempStorage;
        double BestNegLogLike = this->computeNegLogLikelihood(BestSample, TempStorage, false);
        for (const VectorStatic<Dim> &Sample: Samples)
        {
          double NegLogLike = this->computeNegLogLikelihood(Sample, TempStorage, false);
          if (NegLogLike < BestNegLogLike)
          {
            BestSample = Sample;
            BestNegLogLike = NegLogLike;
          }
        }

        /** TODO: optimize afterwards for better precision*/

        return BestSample;
      }


      /** special function for pseudo range stuff */
      VectorStatic<Dim> removeOffset();
      VectorStatic<Dim> removeOffsetLegacy();
      void removeGivenOffset(const VectorStatic<Dim> &Offset);

      void removeMean()
      {
        for (int i = 0; i < static_cast<int>(_Mixture.size()); ++i)
        {
          _Mixture.at(i).setMean(MatrixStatic<Dim, 1>::Zero());
        }
      }

      /** sort for better readability */
      void sortComponentsByMean()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByMean<Dim>);
      }

      void sortComponentsByAbsMean()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByAbsMean<Dim>);
      }

      void sortComponentsByWeight()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByWeight<Dim>);
      }

      void sortComponentsByMode()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByMode<Dim>);
      }

      void sortComponentsByLOSness()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByLOSness<Dim>);
      }

      /** export GMM into state data for logging */
      Data exportToStateData(double Timestamp);

    private:
      std::vector<GaussianComponent<Dim>> _Mixture;
  };
}

#endif // GAUSSIANMIXTURE_H
