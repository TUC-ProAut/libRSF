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

#include "Data.h"

namespace libRSF
{
  /** contructors */
  Data::Data()
  {
    this->_Config = &GlobalDataConfig;
  }

  Data::Data(std::string Input)
  {
    this->_Config = &GlobalDataConfig;
    this->constructFromString(Input);
  }

  Data::Data(DataType Type, double Timestamp)
  {
    this->_Config = &GlobalDataConfig;
    this->constructEmpty(Type, Timestamp);
  }

  double Data::getTimestamp() const
  {
    return this->getValue(DataElement::Timestamp)(0);
  }

  Vector Data::getMean() const
  {
    return this->getValue(DataElement::Mean);
  }

  Matrix Data::getCovarianceMatrix() const
  {
    if (this->checkElement(DataElement::Covariance))
    {
      /** map vector to matrix */
      Vector CovVect = this->getValue(DataElement::Covariance);
      MatrixRef<double, Dynamic, Dynamic> Cov(CovVect.data(), sqrt(CovVect.size()), sqrt(CovVect.size()));
      return Cov;
    }
    else if (this->checkElement(DataElement::CovarianceDiagonal))
    {
      return this->getCovarianceDiagonal().asDiagonal();
    }
    else
    {
      PRINT_ERROR("Data has no covariance element!");
      return Vector();
    }
  }

  Vector Data::getCovarianceDiagonal() const
  {
    if (this->checkElement(DataElement::Covariance))
    {
      return this->getCovarianceMatrix().diagonal();
    }
    else if (this->checkElement(DataElement::CovarianceDiagonal))
    {
      return this->getValue(DataElement::CovarianceDiagonal);
    }
    else
    {
      PRINT_ERROR("Data has no covariance element!");
      return Vector();
    }
  }

  Vector Data::getStdDevDiagonal() const
  {
    if (this->checkElement(DataElement::Covariance))
    {
      return this->getCovarianceMatrix().diagonal().cwiseSqrt();
    }
    else if (this->checkElement(DataElement::CovarianceDiagonal))
    {
      return this->getCovarianceDiagonal().cwiseSqrt();
    }
    else
    {
      PRINT_ERROR("Data has no covariance element!");
      return Vector();
    }
  }

  double* Data::getMeanPointer()
  {
    return this->getDataPointer(DataElement::Mean);
  }

  double const* Data::getMeanPointerConst()
  {
    return this->getDataPointer(DataElement::Mean);
  }

  void Data::setMean(const Vector Mean)
  {
    this->setValue(DataElement::Mean, Mean);
  }

  void Data::setTimestamp(const double Timestamp)
  {
    this->setValueScalar(DataElement::Timestamp, Timestamp);
  }

  void Data::setCovarianceDiagonal(const Vector Cov)
  {
    this->setValue(DataElement::CovarianceDiagonal, Cov);
  }

  void Data::setCovarianceMatrix(const Vector Cov)
  {
    /** we expect a row-major vector representation of the actual matrix here */
    this->setValue(DataElement::Covariance, Cov);
  }

  void Data::setStdDevDiagonal(const Vector StdDev)
  {
    if (this->checkElement(DataElement::Covariance) && this->getValue(DataElement::Covariance).size() == 1)
    {
      this->setValue(DataElement::Covariance, StdDev.array().square());
    }
    else
    {
      this->setValue(DataElement::CovarianceDiagonal, StdDev.array().square());
    }
  }
}
