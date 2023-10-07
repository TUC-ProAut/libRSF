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

#include "error_models/GaussianMixture.h"

namespace libRSF
{
  template<>
  void GaussianMixture<1>::addDiagonal(Vector Mean, Vector StdDev, Vector Weight)
  {
    GaussianComponent<1> Gaussian;
    if (StdDev.size() == Mean.size() && StdDev.size() == Weight.size())
    {
      for(Index nDim = 0; nDim < StdDev.size(); ++nDim)
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
  GaussianMixture<1>::GaussianMixture(Vector Mean, Vector StdDev, Vector Weight)
  {
    addDiagonal(Mean, StdDev, Weight);
  }

  /** legacy version to reproduce the results of ICRA2019 and IV 2019 code */
  template<>
  Vector1 GaussianMixture<1>::removeOffsetLegacy()
  {
    const int NumberOfComponents = this->getNumberOfComponents();
    Vector1 MeanLOS;

    this->sortComponentsByWeight();
    double MinimumWeight = std::min(Mixture_.at(0).getWeight()(0), 0.2);

    /** remove offset of the first "LOS" component */
    this->sortComponentsByMean();
    for(int i = 0; i < NumberOfComponents; ++i)
    {
      if(Mixture_.at(i).getWeight()(0) >= MinimumWeight)
      {
        MeanLOS = Mixture_.at(i).getMean();
        break;
      }
    }

    this->removeGivenOffset(MeanLOS);
    return MeanLOS;
  }

  /** improved version */
  template<>
  Vector1 GaussianMixture<1>::removeOffset()
  {
    const int NumberOfComponents = this->getNumberOfComponents();
    const double MinimumWeight = 1.0 / Mixture_.size()*0.8;

    /** remove offset of the first "LOS" component */
    this->sortComponentsByMean();
    Vector1 MeanLOS = Vector1::Zero();
    for(int i = 0; i < NumberOfComponents; ++i)
    {
      if(Mixture_.at(i).getWeight()(0) >= MinimumWeight)
      {
        MeanLOS = Mixture_.at(i).getMean();
        break;
      }
    }

    this->removeGivenOffset(MeanLOS);
    return MeanLOS;
  }

  template <>
  Data GaussianMixture<1>::exportToStateData(double Timestamp)
  {
    const int NumberOfComponents = this->getNumberOfComponents();

    /** save to vectors */
    Vector Means(NumberOfComponents);
    Vector Covs(NumberOfComponents);
    Vector Weights(NumberOfComponents);
    for(int i = 0; i < NumberOfComponents; ++i)
    {
      Means(i) = Mixture_.at(i).getMean()[0];
      Covs(i) = Mixture_.at(i).getCovariance()[0];
      Weights(i) = Mixture_.at(i).getWeight()[0];
    }

    Data GMMState(DataType::GMM1, Timestamp);
    GMMState.setMean(Means);
    GMMState.setValue(libRSF::DataElement::Covariance, Covs);
    GMMState.setValue(libRSF::DataElement::Weight, Weights);
    return GMMState;
  }
}
