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

}
