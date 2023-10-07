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
 * @file Data.h
 * @author Tim Pfeifer
 * @date 21.11.2016
 * @brief Class which has the capability to represent state or measurement data.
 * @copyright GNU Public License.
 *
 */

#ifndef DATA_H
#define DATA_H

#include "DataGeneric.h"
#include "Types.h"

namespace libRSF
{
  class Data: public DataGeneric<DataType, DataElement>
  {
    public:

      /** standard constructor */
      Data();
      ~Data() override = default;

      /** default constructor for new empty data */
      Data(DataType Type, double Timestamp);

      /** string interface for files */
      explicit Data(const std::string& Input);

      /** specific getters */
      [[nodiscard]] double getTimestamp() const;
      [[nodiscard]] Vector getMean() const;
      [[nodiscard]] Matrix getCovarianceMatrix() const;
      [[nodiscard]] Vector getCovarianceDiagonal() const;
      [[nodiscard]] Vector getStdDevDiagonal() const;

      /** specific pointer getters */
      double* getMeanPointer();
      double const * getMeanPointerConst();

      /** specific setters */
      void setMean(const Vector& Mean);
      void setTimestamp(double Timestamp);
      void setCovarianceDiagonal(const Vector& Cov);
      void setStdDevDiagonal(const Vector& StdDev);
      void setCovariance(const Vector& Cov);
      void setCovarianceMatrix(const Matrix& Cov);

    private:

  };
}

#endif // DATA_H
