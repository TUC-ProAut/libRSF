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
 * @file GNSS.h
 * @author Tim Pfeifer
 * @date 16.04.2019
 * @brief GNSS related helper functions like coordinate transformations.
 * @copyright GNU Public License.
 *
 */

#ifndef GNSS_H
#define GNSS_H

#include "Messages.h"
#include "SensorDataSet.h"
#include "StateDataSet.h"
#include "VectorMath.h"
#include "factors/PseudorangeFactor.h"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <Eigen/Dense>

namespace libRSF
{
  /** convert points between the global ECEF system and a local ENU frame */
  class TangentPlaneConverter
  {
  public:
    TangentPlaneConverter();
    explicit TangentPlaneConverter(Vector3 TangentPoint);
    ~TangentPlaneConverter(){};

    /** set local reference point */
    void setTangentPoint(Vector3 TangentPoint);

    /** check status */
    bool isInitialized();

    /** element-wise interface */
    void convertMeasurementToLocal(Data &Measurement);
    void convertMeasurementToGlobal(Data &Measurement);
    void convertStateToLocal(Data &State);
    void convertStateToGlobal(Data &State);

    /** batch interface */
    void convertAllPseudorangesToLocal(SensorDataSet &Measurements);
    void convertAllStatesToLocal(StateDataSet &States, std::string ID);
    void convertAllStatesToGlobal(StateDataSet &States, std::string ID);

  private:
    /** convert mean only */
    Vector3 convertToLocal(const Vector3 &GlobalPoint) const;
    Vector3 convertToGlobal(const Vector3 &LocalPoint) const;

    /** convert mean and covariance */
    void convertToLocal(const Vector3  &GlobalPoint,
                        const Matrix33 &GlobalCov,
                        Vector3  &LocalPoint,
                        Matrix33 &LocalCov) const;

    void convertToGlobal(const Vector3  &LocalPoint,
                         const Matrix33 &LocalCov,
                         Vector3  &GlobalPoint,
                         Matrix33 &GlobalCov) const;

    bool                              _isInitialized;
    Vector3                           _TangentPoint;

    GeographicLib::LocalCartesian     _LocalProjection;
    const GeographicLib::Geocentric   _Earth = GeographicLib::Geocentric::WGS84();
  };


}

#endif // GNSS_H
