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
#include "../FactorGraphConfig.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/SpecialFunctions>

#include <algorithm>

namespace libRSF
{
  template <int Dim>
  class GaussianMixture
  {
    public:

      using ErrorMatType = MatrixT<double, Dim, Dynamic>;

      /** config for mixture estimation */
      struct EstimationConfig
      {
        using EstimationAlgorithmType = ErrorModelTuningType;
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
        double PriorDirichletConcentration = 1e-3;

        double PriorNormalInfoScaling = 1e-4;
        VectorStatic<Dim> PriorNormalMean = VectorStatic<Dim>::Zero();

        double PriorWishartDOF = Dim;
        MatrixStatic<Dim, Dim> PriorWishartScatter = MatrixStatic<Dim, Dim>::Identity();

        /** remove components if the number of assigned samples is too low */
        bool RemoveSmallComponents = false;
        /** the minimal number of samples that have to support a component*/
        double MinSamplePerComponent = Dim + 1;

        /** check the Bhattacharyya distance between all components to merge similar ones */
        bool MergeSimilarComponents = false;
        double MergingThreshold = 0.1;
      };

      /** Bayesian representation of the parameter estimation problem */
      struct BayesianState
      {
        std::vector<double> AlphaWeight;
        std::vector<double> Weight;

        VectorVectorSTL<Dim> MeanMean;
        std::vector<double> BetaMean;
        MatrixVectorSTL<Dim, Dim> InfoMean;

        std::vector<double> NuInfo;
        MatrixVectorSTL<Dim, Dim> WInfo;

        Matrix Responsibilities;
      };

      /** default constructor */
      GaussianMixture() = default;
      virtual ~GaussianMixture() = default;

      /** convenient constructor from external config struct*/
      explicit GaussianMixture(const FactorGraphConfig::GaussianMixtureConfig &Config)
      {
        switch (Config.TuningType)
        {
          case ErrorModelTuningType::None:
            {
              /** find number of components */
              const int NumComp = Config.Weight.size();

              /** add components */
              for(int n = 0; n < NumComp; n++)
              {
                GaussianComponent<Dim> Comp;
                MatrixStatic<Dim, Dim> Cov;
                VectorStatic<Dim> Mean;

                Mean = Config.Mean.template segment<Dim>(n*Dim);
                Cov << Config.StdDev.template segment<Dim*Dim>(n*Dim*Dim).transpose(); /** transpose for row-first*/
                Cov = Cov.transpose()*Cov;


                Comp.setParamsCovariance(Cov, Mean, Config.Weight.template segment<1>(n));
                this->addComponent(Comp);
              }
            }
            break;

          case ErrorModelTuningType::EM:
          case ErrorModelTuningType::EM_MAP:
          case ErrorModelTuningType::VBI:
          case ErrorModelTuningType::VBI_Full:
            this->initSpread(Config.NumberComponents, Config.BaseStandardDeviation);
            break;

          default:
            PRINT_ERROR("Error model tuning type not handled: ", Config.TuningType);
            break;
        }
      }

      static EstimationConfig ConvertConfig(const FactorGraphConfig::GaussianMixtureConfig &Config)
      {
        EstimationConfig NewConfig;

        /** copy type */
        NewConfig.EstimationAlgorithm = Config.TuningType;

        /** copy prior parameter*/
        switch (Config.TuningType)
        {
          case ErrorModelTuningType::EM:
            break;

          case ErrorModelTuningType::EM_MAP:
          case ErrorModelTuningType::VBI_Full:
            NewConfig.PriorDirichletConcentration = Config.PriorDirichletConcentration;
            NewConfig.PriorNormalInfoScaling = Config.PriorNormalInfoScaling;
            NewConfig.PriorWishartDOF = Config.PriorWishartDOF;
            break;

          case ErrorModelTuningType::VBI:
            NewConfig.PriorNormalInfoScaling = Config.PriorNormalInfoScaling;
            NewConfig.PriorWishartDOF = Config.PriorWishartDOF;
            break;

          default:
            PRINT_ERROR("Error model tuning type not handled: ", Config.TuningType);
            break;
        }

        if(Config.IncrementalTuning)
        {
          NewConfig.RemoveSmallComponents = true;
        }
        else
        {
          NewConfig.RemoveSmallComponents = false;
        }

        return NewConfig;
      }

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

        /** set a good covariance prior, that represents the data's uncertainty*/
        EstimationConfig ModifiedConfig = Config;
        if (Config.EstimationAlgorithm != ErrorModelTuningType::EM)
        {
          ModifiedConfig.PriorWishartScatter = EstimateSampleCovariance(DataMatrix) * Config.PriorWishartDOF;
        }

        /** init*/
        BayesianState VBIState;
        bool ReachedMaxIteration = false;
        bool Converged = false;
        bool Merged = false;
        bool Pruned = false;
        double LikelihoodSumOld = 0;
        double LikelihoodSum = 0;
        int k = 1;

        /** iterate until convergence */
        while ((!Converged && !ReachedMaxIteration) || Merged || Pruned)
        {
          /** init required variables */
          int M = Mixture_.size();

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
              /** multivariate VBI, without Dirichlet prior*/
              if (k == 1)
              {
                /** first likelihood is not variational */
                this->computeProbability(DataMatrix, VBIState.Responsibilities);
              }
              LikelihoodSum = this->doVariationalStep(DataMatrix, VBIState, ModifiedConfig);
            }
            break;

            case ErrorModelTuningType::VBI_Full:
            {
              /** multivariate VBI, with Dirichlet prior*/
              if (k == 1)
              {
                /** first likelihood is not variational */
                this->computeProbability(DataMatrix, VBIState.Responsibilities);
              }
              LikelihoodSum = this->doVariationalStepFull(DataMatrix, VBIState, ModifiedConfig);
            }
            break;

            default:
              PRINT_ERROR("Wrong mixture estimation algorithm type!");
              break;
          }

          /** post-process mixture (only for EM, for VBI it is done internally)*/
          if (Config.EstimationAlgorithm == ErrorModelTuningType::EM ||
              Config.EstimationAlgorithm == ErrorModelTuningType::EM_MAP)
          {
            /** remove components with a low weight */
            if (Config.RemoveSmallComponents)
            {
              Pruned = this->prunMixture(std::min(Config.MinSamplePerComponent / N, 1.0));
            }
            else
            {
              Pruned = this->prunMixture(0.0);
            }

            /** merge components that are close to each other */
            if (Config.MergeSimilarComponents)
            {
              Merged = this->reduceMixture(Config.MergingThreshold);
            }
            else
            {
              Merged = false;
            }
          }

          /** check for convergence */
          const double LikelihoodChange = std::abs(LikelihoodSum - LikelihoodSumOld) / LikelihoodSum;
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

        /** take expectations from Bayesian results */
        if (Config.EstimationAlgorithm == ErrorModelTuningType::VBI||
            Config.EstimationAlgorithm == ErrorModelTuningType::VBI_Full)
        {
          this->extractParameterFromBayes(VBIState);
        }

        /** check if any GMM parameter is consistent after the estimation (only in debug mode)*/
        bool GMMConsistency = true;
#ifndef NDEBUG
        for (int m = 0; m < static_cast<int>(Mixture_.size()); m++)
        {
          if (Mixture_.at(m).checkParameters() == false)
          {
            PRINT_ERROR("Check of GMM component ", m+1, " of ", Mixture_.size(), " failed!");
            GMMConsistency = false;
          }
        }

        PRINT_LOGGING("GMM estimation started with ", N, " samples.");
        PRINT_LOGGING("GMM estimation ended with ", Mixture_.size(), " components, after ", k - 1, " iterations.");
        this->printParameter();
#endif

        return GMMConsistency;
      }

      /** init with increasing uncertainties */
      void initSpread(const int Components, const double BaseStdDev, const bool IsAsymmetric = false)
      {
        libRSF::GaussianComponent<Dim> Component;

        MatrixStatic<Dim, 1> Mean = MatrixStatic<Dim, 1>::Zero();
        const MatrixStatic<1, 1> Weight = MatrixStatic<1, 1>::Ones() / Components;
        const MatrixStatic<Dim, Dim> SqrtInfo = MatrixStatic<Dim, Dim>::Identity() * (1.0 / BaseStdDev);

        this->clear();
        for (int nComponent = 0; nComponent < static_cast<int>(Components); ++nComponent)
        {
          if(nComponent > 0 && IsAsymmetric)
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
        Mixture_.emplace_back(Gaussian);
      }

      /** for distribution-to-distribution models */
      void inflateWithCov(MatrixStatic<Dim, Dim> Cov)
      {
        for (int i = 0; i < static_cast<int>(Mixture_.size()); ++i)
        {
          MatrixStatic<Dim, Dim> CovNew;

          CovNew = Cov + Mixture_.at(i).getCovariance();

          Mixture_.at(i).updateCovariance(CovNew);
        }
      }

      void removeLastComponent()
      {
        Mixture_.pop_back();
      }

      void clear()
      {
        Mixture_.clear();
      }

      [[nodiscard]] int getNumberOfComponents() const
      {
        return Mixture_.size();
      }

      void getMixture(std::vector<GaussianComponent<Dim>> &Mixture)
      {
        Mixture = Mixture_;
      }

      void computeLikelihood(const ErrorMatType &DataVector, Matrix &Likelihood) const
      {
        const int M = Mixture_.size(); /**< number of components */
        const int N = DataVector.cols(); /**< number of data samples */

        /** adapt size of output matrix */
        Likelihood.resize(M, N);

        /** loop over components*/
        for (int m = 0; m < M; ++m)
        {
          Likelihood.row(m) = Mixture_.at(m).computeLikelihood(DataVector);
        }

        /** remove NaNs */
        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);
      }

      double computeProbability(const ErrorMatType &DataVector, Matrix &Probability) const
      {
        const int M = Mixture_.size(); /**< number of components */
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
        const int M = Mixture_.size(); /**< number of components */
        const int N = DataVector.cols(); /**< number of data samples */

        /** adapt size of output matrix */
        NegLogLikelihood.resize(M, N);

        /** loop over components*/
        for (int m = 0; m < M; ++m)
        {
          NegLogLikelihood.row(m) = Mixture_.at(m).computeNegLogLikelihood(DataVector, EvalAsError);
        }

        /** remove NaNs */
        NegLogLikelihood = (NegLogLikelihood.array().isFinite()).select(NegLogLikelihood, 1e100);

        return NegLogLikelihood.sum();
      }

      void extractParameterFromBayes(const BayesianState &State)
      {
        /** delete old mixture */
        Mixture_.clear();

        /** extract parameter component-wise */
        const int GMMSize = State.Weight.size();
        for (int k = 0; k < GMMSize; ++k)
        {
          libRSF::GaussianComponent<Dim> Component;
          Component.setParamsInformation(State.NuInfo.at(k)*State.WInfo.at(k),
                                         -State.MeanMean.at(k),
                                         (Vector1() << State.Weight.at(k)).finished());
          this->addComponent(Component);
        }
      }

      double doVariationalStep(const ErrorMatType &DataMatrix,
                               BayesianState &VBIState,
                               const EstimationConfig &Config) const
      {

        /** Implementation of:
         * Corduneanu, A. & Bishop, C.
         * Variational Bayesian Model Selection for Mixture Distributions
         * Proc. of Intl. Conf. on Artificial Intelligence and Statistics (AISTATS)
         * 2001
         */

        /** rename hyperprior variables */
        const MatrixStatic<Dim,Dim> V0 = Config.PriorWishartScatter;
        const MatrixStatic<Dim,Dim> Beta0 = Config.PriorNormalInfoScaling * MatrixStatic<Dim,Dim>::Identity();
        const VectorStatic<Dim> Mean0 = Config.PriorNormalMean;
        const double Nu0 = Config.PriorWishartDOF;

        /** adapt to current size */
        const int SampleSize = DataMatrix.cols();
        if (VBIState.Weight.size() < VBIState.Responsibilities.rows())
        {
          for (int n = VBIState.Weight.size(); n < VBIState.Responsibilities.rows(); ++n)
          {
            VBIState.Weight.push_back(this->Mixture_.at(n).getWeight()(0));
            VBIState.InfoMean.push_back(Beta0); /**< will be overwritten */
            VBIState.MeanMean.push_back(Mean0); /**< will be overwritten */
            VBIState.NuInfo.push_back(Nu0);
            VBIState.WInfo.push_back(Inverse(V0));
          }
        }
        const int GMMSize = VBIState.Weight.size();

        /** Expectation step */

        /** pre-calculate some multi-use variables */
        const Vector SumLike = VBIState.Responsibilities.rowwise().sum();
        const MatrixStatic<Dynamic, Dim> SumLikeX = VBIState.Responsibilities * DataMatrix.transpose();

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
            SumLikeXXComp += XX.at(n) * VBIState.Responsibilities(m,n);
          }
          SumLikeXX.push_back(SumLikeXXComp);
        }

        /** iterate over GMM components */
        for (int m = 0; m < GMMSize; ++m)
        {
          /** <T_i> */
          MatrixStatic<Dim,Dim> InfoEx = VBIState.NuInfo.at(m) * VBIState.WInfo.at(m);

          /** update mean posterior */
          VBIState.InfoMean.at(m) = Beta0 + InfoEx * SumLike(m);

          if (Config.EstimateMean)
          {
            VBIState.MeanMean.at(m) = Inverse(VBIState.InfoMean.at(m)) * InfoEx * SumLikeX.row(m).transpose();
          }

          /** update variance posterior */
          VBIState.NuInfo.at(m) = Nu0 + SumLike(m);

          /** <Mu_i*Mu_i^T> */
          const MatrixStatic<Dim,Dim> MuMuEx = Inverse(VBIState.InfoMean.at(m)) + VBIState.MeanMean.at(m) * VBIState.MeanMean.at(m).transpose();

          MatrixStatic<Dim,Dim> SumXPMu = MatrixStatic<Dim,Dim>::Zero();
          for (int n = 0; n < SampleSize; n++)
          {
            SumXPMu += DataMatrix.col(n) * VBIState.Responsibilities(m,n) * VBIState.MeanMean.at(m).transpose();
          }

          VBIState.WInfo.at(m) = (V0
                                 + SumLikeXX.at(m)
                                 - SumXPMu
                                 - VBIState.MeanMean.at(m) * SumLikeX.row(m)
                                 + MuMuEx * SumLike(m)).inverse();

          /** variational likelihood */
          /** <ln|T_i|> */
          double LogInfoEx = 0;
          for (int d = 1; d <= Dim; d++)
          {
            LogInfoEx += Eigen::numext::digamma(0.5 * (VBIState.NuInfo.at(m) + 1 - d));
          }
          LogInfoEx += Dim*log(2.0) + log(VBIState.WInfo.at(m).determinant());

          /** update <T_i> */
          InfoEx = VBIState.NuInfo.at(m) * VBIState.WInfo.at(m);

          /** <Mu_i> */
          const VectorStatic<Dim> MeanEx = VBIState.MeanMean.at(m);

          VBIState.Responsibilities.row(m).fill(0.5 * LogInfoEx + log(VBIState.Weight.at(m)));
          for (int n = 0; n < SampleSize; n++)
          {
            VBIState.Responsibilities(m,n) -= 0.5 * (InfoEx * (XX.at(n)
                                              - MeanEx * DataMatrix.col(n).transpose()
                                              - DataMatrix.col(n) * MeanEx.transpose()
                                              + MuMuEx) ).trace();
          }
          VBIState.Responsibilities.row(m) = VBIState.Responsibilities.row(m).array().exp();
        }

        /** "Maximization step" --> update probability */
        /** remove NaN */
        VBIState.Responsibilities = (VBIState.Responsibilities.array().isFinite()).select(VBIState.Responsibilities, 0.0);

        /** calculate relative likelihood  */
        double LikelihoodSum = VBIState.Responsibilities.sum();
        for (int n = 0; n < static_cast<int>(VBIState.Responsibilities.cols()); ++n)
        {
          VBIState.Responsibilities.col(n) /= VBIState.Responsibilities.col(n).sum();
        }

        /** remove NaN again (occur if sum of likelihoods is zero) */
        VBIState.Responsibilities = (VBIState.Responsibilities.array().isFinite()).select(VBIState.Responsibilities, 1.0 / GMMSize);

        /** calculate weights */
        for (int m = 0; m < GMMSize; ++m)
        {
          VBIState.Weight.at(m) = VBIState.Responsibilities.row(m).sum() / VBIState.Responsibilities.cols();
        }

        /** remove useless or degenerated components */
        for (int m = GMMSize-1; m >=0 ; --m)
        {
          if(((VBIState.Weight.at(m) < (Config.MinSamplePerComponent / SampleSize) && Config.RemoveSmallComponents ) || std::isnan(VBIState.Weight.at(m))) && VBIState.Weight.size() > 1)
          {
            /** remove component from posterior states */
            VBIState.NuInfo.erase(VBIState.NuInfo.begin() + m);
            VBIState.WInfo.erase(VBIState.WInfo.begin() + m);
            VBIState.InfoMean.erase(VBIState.InfoMean.begin() + m);
            VBIState.MeanMean.erase(VBIState.MeanMean.begin() + m);
            VBIState.Weight.erase(VBIState.Weight.begin() + m);

            /** remove row from probability matrix */
            Matrix ProbTrans = VBIState.Responsibilities.transpose();
            RemoveColumn(ProbTrans,m);
            VBIState.Responsibilities = ProbTrans.transpose();

            /** enforce an additional iteration after removal by resetting likelihood */
            LikelihoodSum = 1e40;
          }
        }

        return LikelihoodSum;
      }

      double doVariationalStepFull(const ErrorMatType &DataMatrix,
                                   BayesianState &VBIState,
                                   const EstimationConfig &Config) const
      {
        /** Implementation based on:
        * Christopher M. Bishop
        * Pattern Recognition and Machine Learning, Section 10.2
        * 2006
        */

        /** copy hyper-priors */
        const MatrixStatic<Dim,Dim> W0Inv = Config.PriorWishartScatter;
        const double Beta0 = Config.PriorNormalInfoScaling;
        const double Nu0 = Config.PriorWishartDOF;
        const double Alpha0 = Config.PriorDirichletConcentration;
        const VectorStatic<Dim> Mean0 = Config.PriorNormalMean;

        /** adapt state to current size */
        const int SampleSize = DataMatrix.cols();
        if (VBIState.AlphaWeight.size() < VBIState.Responsibilities.rows())
        {
          for (int n = VBIState.AlphaWeight.size(); n < VBIState.Responsibilities.rows(); ++n)
          {
            VBIState.AlphaWeight.push_back(Alpha0);
            VBIState.MeanMean.push_back(Mean0);
            VBIState.BetaMean.push_back(Beta0);
            VBIState.NuInfo.push_back(Nu0);
            VBIState.WInfo.push_back(Inverse(Config.PriorWishartScatter));
            VBIState.Weight.push_back(this->Mixture_.at(n).getWeight()(0));
          }
        }
        const int GMMSize = VBIState.AlphaWeight.size();

        /** pre-calculate useful variables */
        const Vector N = VBIState.Responsibilities.rowwise().sum();
        const double NSum = N.sum();
        const Vector NInv = (N.array().inverse().isFinite()).select(N.array().inverse(), 0.0); /**< catch NaN */
        MatrixStatic<Dynamic, Dim> MeanX (GMMSize, Dim);
        for (int k = 0; k < GMMSize; k++)
        {
          MeanX.row(k) = NInv(k) * (DataMatrix * VBIState.Responsibilities.row(k).asDiagonal()).rowwise().sum();
        }

        MatrixVectorSTL<Dim, Dim> S;
        for (int k = 0; k < GMMSize; k++)
        {
          MatrixStatic<Dim,Dim> Sum = MatrixStatic<Dim,Dim>::Zero();
          for (int n = 0; n < SampleSize; n++)
          {
            Sum += VBIState.Responsibilities(k,n)
                   * ((DataMatrix.col(n) - MeanX.row(k).transpose())
                      * (DataMatrix.col(n) - MeanX.row(k).transpose()).transpose());
          }
          S.push_back(NInv(k) * Sum);
        }

        /** update posteriors */
        double SumAlpha = 0;
        for (int k = 0; k < GMMSize; k++)
        {
          VBIState.AlphaWeight.at(k) = Alpha0 + N(k);
          SumAlpha += VBIState.AlphaWeight.at(k);

          VBIState.BetaMean.at(k) = Beta0 + N(k);
          if (Config.EstimateMean)
          {
            VBIState.MeanMean.at(k) = 1.0/VBIState.BetaMean.at(k) * (Beta0*Mean0 + N(k)*MeanX.row(k).transpose());
          }

          const MatrixStatic<Dim,Dim> WInfoInv = W0Inv
                                               + N(k)*S.at(k)
                                               + Beta0*N(k)/(Beta0 + N(k)) * (MeanX.row(k).transpose() - Mean0)*(MeanX.row(k).transpose() - Mean0).transpose();

          VBIState.WInfo.at(k) = Inverse(WInfoInv);
          VBIState.NuInfo.at(k) = Nu0 + N(k);

          /** update expected weight */
          VBIState.Weight.at(k) = (Alpha0 + N(k))/(GMMSize*Alpha0 + NSum);
        }

        /** evaluate expectations */
        Vector ExpLnInfo(GMMSize);
        Vector ExpLnWeight(GMMSize);
        for (int k = 0; k < GMMSize; k++)
        {
          double Sum = 0;
          for (int d = 1; d <= Dim; d++)
          {
            Sum += Eigen::numext::digamma(0.5 * (VBIState.NuInfo.at(k) + 1 - d));
          }
          ExpLnInfo(k) = Sum + Dim*log(2.0) + log(VBIState.WInfo.at(k).determinant());
          ExpLnWeight(k) = Eigen::numext::digamma(VBIState.AlphaWeight.at(k))
                           - Eigen::numext::digamma(SumAlpha);
        }

        /** evaluate responsibilities */
        for (int k = 0; k < GMMSize; k++)
        {
          for (int n = 0; n < SampleSize; n++)
          {
            VBIState.Responsibilities(k,n) = exp(ExpLnWeight(k)
                                                 + ExpLnInfo(k)/2.0
                                                 - Dim/(2.0*VBIState.BetaMean.at(k))
                                                 - VBIState.NuInfo.at(k)/2.0
                                                 * ((DataMatrix.col(n) - VBIState.MeanMean.at(k)).transpose() * VBIState.WInfo.at(k)).dot(
                                                   (DataMatrix.col(n) - VBIState.MeanMean.at(k))));
          }
        }

        /** catch numerical issues with the exp() */
        VBIState.Responsibilities = (VBIState.Responsibilities.array().isFinite()).select(VBIState.Responsibilities, 0.0);

        /** save likelihood before(!) it is normalized as probability */
        double LikelihoodSum = VBIState.Responsibilities.sum();

        /** normalize */
        for (int n = 0; n < static_cast<int>(VBIState.Responsibilities.cols()); ++n)
        {
          VBIState.Responsibilities.col(n) /= VBIState.Responsibilities.col(n).sum();
        }

        /** remove NaN (occur if sum of likelihoods is zero) */
        VBIState.Responsibilities = (VBIState.Responsibilities.array().isFinite()).select(VBIState.Responsibilities, 1.0 / GMMSize);

        /** remove useless or degenerated components */
        for (int k = GMMSize-1; k >=0 ; --k)
        {

          if(((VBIState.Weight.at(k) < (Config.MinSamplePerComponent / SampleSize) && Config.RemoveSmallComponents ) || std::isnan(VBIState.Weight.at(k))) && VBIState.Weight.size() > 1)
          {
            /** remove component from posterior states */
            VBIState.AlphaWeight.erase(VBIState.AlphaWeight.begin() + k);
            VBIState.MeanMean.erase(VBIState.MeanMean.begin() + k);
            VBIState.BetaMean.erase(VBIState.BetaMean.begin() + k);
            VBIState.NuInfo.erase(VBIState.NuInfo.begin() + k);
            VBIState.WInfo.erase(VBIState.WInfo.begin() + k);
            VBIState.Weight.erase(VBIState.Weight.begin() + k);

            /** remove row from probability matrix */
            Matrix RespTrans = VBIState.Responsibilities.transpose();
            RemoveColumn(RespTrans, k);
            VBIState.Responsibilities = RespTrans.transpose();

            /** enforce an additional iteration after removal by resetting likelihood */
            LikelihoodSum = 1e40;
          }
        }

        return LikelihoodSum;
      }

      void computeMixtureParameters(const ErrorMatType &DataVector, const Matrix &Likelihood, const EstimationConfig &Config)
      {
        const int M = Mixture_.size();
        for (int m = 0; m < M; m++)
        {
          Mixture_.at(m).estimateParameters(DataVector, Likelihood.row(m), Config.EstimateMean);
        }
      }

      void computeMixtureParametersMAP(const ErrorMatType &DataVector, const Matrix &Likelihood, const EstimationConfig &Config)
      {

        /** sum over components */
        const int M = Mixture_.size();
        const double DirichletSum = (Config.PriorDirichletConcentration - 1) * M;

        for (int m = 0; m < M; m++)
        {
          Mixture_.at(m).estimateParametersMAP(DataVector,
                                               Likelihood.row(m),
                                               Config.PriorDirichletConcentration,
                                               DirichletSum,
                                               Config.PriorNormalInfoScaling,
                                               Config.PriorNormalMean,
                                               Config.PriorWishartScatter,
                                               Config.PriorWishartDOF,
                                               Config.EstimateMean);
        }
      }

      /** remove components, that are to small */
      bool prunMixture(const double MinWeight)
      {
        bool Pruned = false;
        for (int m = Mixture_.size() - 1; m >= 0; m--)
        {
          if ((Mixture_.at(m).getWeight()(0) < MinWeight ||
               Mixture_.at(m).getWeight()(0) == 0.0)
              &&
              Mixture_.size() > 1)
          {
            Mixture_.erase(Mixture_.begin() + m);
            Pruned = true;
          }
        }

        return Pruned;
      }

      /** merge components that are identical */
      bool reduceMixture(const double BhattacharyyaLimit)
      {
        bool PerformedMerge = false;
        int M = Mixture_.size();

        for (int m1 = 0; m1 < M - 1; ++m1)
        {
          for (int m2 = m1 + 1; m2 < M; ++m2)
          {
            /** calculate metric */
            double d = CalculateBhattacharyyaDistance(Mixture_.at(m1),
                                                      Mixture_.at(m2));

            /** perform merge if the distance exceed a specific tuning parameter*/
            if (exp(-d) > BhattacharyyaLimit)
            {
              Mixture_.at(m1) = MergeGaussians<Dim>(Mixture_.at(m1), Mixture_.at(m2));
              Mixture_.erase(Mixture_.begin() + m2);

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
        for (int n = 0; n < static_cast<int>(Mixture_.size()); ++n)
        {
          PRINT_LOGGING("Component ", n + 1, ":   ",
                        Mixture_.at(n).getMean().transpose(), "       ",
                        Mixture_.at(n).getSqrtInformation().diagonal().cwiseInverse().transpose(), "  ",
                        Mixture_.at(n).getWeight());
        }
      }

      /** query error values for a specific component */
      template <typename T>
      VectorT<T, Dim> getExponentialPartOfComponent(int NumberOfComponent, const VectorT<T, Dim> &Error) const
      {
        return Mixture_.at(NumberOfComponent).getExponentialPart(Error);
      }

      template <typename T>
      double getLinearPartOfComponent(int NumberOfComponent, const VectorT<T, Dim> &Error) const
      {
        return Mixture_.at(NumberOfComponent).getLinearPart(Error);
      }

      [[nodiscard]] double getMaximumOfComponent(int NumberOfComponent) const
      {
        return Mixture_.at(NumberOfComponent).getMaximum();
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
            CurrentNumber = round(Number * this->Mixture_.at(n).getWeight()(0));
          }
          else
          {
            /** fill with last component to avoid round-off */
            CurrentNumber = Number - SampleVector.size();
          }

          /** draw */
          const VectorVectorSTL<Dim> CurrentSamples = this->Mixture_.at(n).DrawSamples(CurrentNumber);

          /** append */
          SampleVector.insert(std::end(SampleVector), std::begin(CurrentSamples), std::end(CurrentSamples));
        }

        return SampleVector;
      }

      /** estimate mode of the GMM */
      VectorStatic<Dim> EstimateMode() const
      {
        /** draw samples */
        const VectorVectorSTL<Dim> Samples = DrawSamples(1e4*Dim);

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
      void removeGivenOffset(const VectorStatic<Dim> &Offset)
      {
        /** remove offset from all components */
        const int NumberOfComponents = this->getNumberOfComponents();
        for(int i = 0; i < NumberOfComponents; ++i)
        {
          Mixture_.at(i).setMean(Mixture_.at(i).getMean() - Offset);
        }
      }

      void removeMean()
      {
        for (int i = 0; i < static_cast<int>(Mixture_.size()); ++i)
        {
          Mixture_.at(i).setMean(MatrixStatic<Dim, 1>::Zero());
        }
      }

      /** sort for better readability */
      void sortComponentsByMean()
      {
        std::sort(Mixture_.begin(), Mixture_.end(), compareByMean<Dim>);
      }

      void sortComponentsByAbsMean()
      {
        std::sort(Mixture_.begin(), Mixture_.end(), compareByAbsMean<Dim>);
      }

      void sortComponentsByWeight()
      {
        std::sort(Mixture_.begin(), Mixture_.end(), compareByWeight<Dim>);
      }

      void sortComponentsByMode()
      {
        std::sort(Mixture_.begin(), Mixture_.end(), compareByMode<Dim>);
      }

      void sortComponentsByLOSness()
      {
        std::sort(Mixture_.begin(), Mixture_.end(), compareByLOSness<Dim>);
      }

      /** export GMM into state data for logging */
      Data exportToStateData(double Timestamp);

    private:
      std::vector<GaussianComponent<Dim>> Mixture_;
  };
}

#endif // GAUSSIANMIXTURE_H
