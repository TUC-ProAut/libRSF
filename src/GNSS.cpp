/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
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

#include "GNSS.h"

namespace libRSF
{
  TangentPlaneConverter::TangentPlaneConverter(): _LocalProjection()
  {
    _isInitialized = false;
  }

  TangentPlaneConverter::TangentPlaneConverter(Vector3 TangentPoint): _LocalProjection()
  {
    setTangentPoint(TangentPoint);
  }

  void TangentPlaneConverter::setTangentPoint(Vector3 TangentPoint)
  {
    /** convert to LLA */
    double Lat0, Lon0, H0;
    _Earth.Reverse(TangentPoint(0), TangentPoint(1), TangentPoint(2), Lat0, Lon0, H0);

    /** init converter */
    _LocalProjection.Reset(Lat0, Lon0, H0);

    _TangentPoint = TangentPoint;
    _isInitialized = true;
  }

  Vector3 TangentPlaneConverter::convertToLocal(const Vector3 &GlobalPoint) const
  {
    /** check for initialization*/
    if(!_isInitialized)
    {
      PRINT_WARNING("Converter is not initialized!");
      return GlobalPoint;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    Vector3 LocalPoint;
    _Earth.Reverse(GlobalPoint(0), GlobalPoint(1), GlobalPoint(2), Lat, Lon, H);
    _LocalProjection.Forward(Lat, Lon, H, X, Y, Z);
    LocalPoint << X, Y, Z;

    return LocalPoint;
  }

  Vector3 TangentPlaneConverter::convertToGlobal(const Vector3 &LocalPoint) const
  {
    /** check for initialization*/
    if(!_isInitialized)
    {
      PRINT_WARNING("Converter is not initialized!");
      return LocalPoint;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    Vector3 GlobalPoint;

    _LocalProjection.Reverse(LocalPoint(0), LocalPoint(1), LocalPoint(2), Lat, Lon, H);
    _Earth.Forward(Lat, Lon, H, X, Y, Z);
    GlobalPoint << X, Y, Z;

    return GlobalPoint;
  }

  void TangentPlaneConverter::convertToLocal(const Vector3   &GlobalPoint,
                                             const Matrix33  &GlobalCov,
                                                   Vector3   &LocalPoint,
                                                   Matrix33  &LocalCov) const
  {
    /** check for initialization*/
    if(!_isInitialized)
    {
      PRINT_WARNING("Converter is not initialized!");
      return;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    std::vector<double> R1(9);
    std::vector<double> R2(9);

    /** calculate trasformation */
    _Earth.Reverse(GlobalPoint(0), GlobalPoint(1), GlobalPoint(2), Lat, Lon, H, R1);
    _LocalProjection.Forward(Lat, Lon, H, X, Y, Z, R2);

    /** convert to eigen */
    Matrix33 R1e(R1.data());
    Matrix33 R2e(R2.data());

    LocalPoint << X, Y, Z;
    LocalCov = R2e*R1e*GlobalCov*(R2e*R1e).transpose();
  }

  void TangentPlaneConverter::convertToGlobal(const Vector3   &LocalPoint,
                                              const Matrix33  &LocalCov,
                                                    Vector3   &GlobalPoint,
                                                    Matrix33  &GlobalCov) const
  {
    /** check for initialization*/
    if(!_isInitialized)
    {
      PRINT_WARNING("Converter is not initialized!");
      return;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    std::vector<double> R1(9);
    std::vector<double> R2(9);

    /** calculate trasformation */
    _LocalProjection.Reverse(LocalPoint(0), LocalPoint(1), LocalPoint(2), Lat, Lon, H, R1);
    _Earth.Forward(Lat, Lon, H, X, Y, Z, R2);


    /** convert to eigen */
    Matrix33 R1e(R1.data());
    Matrix33 R2e(R2.data());

    GlobalPoint << X, Y, Z;
    GlobalCov = R2e*R1e*LocalCov*(R2e*R1e).transpose();
  }


  void TangentPlaneConverter::convertStateToLocal(StateData &State)
  {
    /** check if the state has the right type */
    if(State.getType() != StateType::Point3)
    {
      PRINT_ERROR("Wrong state Type: ", State.getType());
      return;
    }

    /** extract values */
    Vector3 MeanIn = State.getMean();
    Matrix33 CovIn(State.getCovariance().data());

    /** convert */
    Vector3 MeanOut;
    Matrix33 CovOut;
    this->convertToLocal(MeanIn, CovIn, MeanOut, CovOut);

    /** write back */
    Vector9 CovOutVect(CovOut.data());
    State.setMean(MeanOut);
    State.setCovariance(CovOutVect);
  }

  void TangentPlaneConverter::convertStateToGlobal(StateData &State)
  {
    /** check if the state has the right type */
    if(State.getType() != StateType::Point3)
    {
      PRINT_ERROR("Wrong state Type: ", State.getType());
      return;
    }

    /** extract values */
    Vector3 MeanIn = State.getMean();
    Matrix33 CovIn(State.getCovariance().data());

    /** convert */
    Vector3 MeanOut;
    Matrix33 CovOut;
    this->convertToGlobal(MeanIn, CovIn, MeanOut, CovOut);

    /** write back */
    Vector9 CovOutVect(CovOut.data());
    State.setMean(MeanOut);
    State.setCovariance(CovOutVect);
  }

  void TangentPlaneConverter::convertMeasurementToLocal(SensorData &Measurement)
  {
    /** check if the measurement has the right type */
    if(Measurement.getType() != SensorType::Pseudorange3)
    {
      PRINT_ERROR("Wrong sensor Type: ", Measurement.getType());
      return;
    }

    /** remove earth rotation effect */
    Vector3 SatPosGlobal = Measurement.getValue(SensorElement::SatPos);
    Vector1 RelCor;
    RelCor(0) = RelativisticCorrection(_TangentPoint.data(), SatPosGlobal);
    Measurement.setMean(Measurement.getMean() - RelCor);

    /** convert sat pos */
    Measurement.setValue(SensorElement::SatPos, convertToLocal(SatPosGlobal));
  }

  void TangentPlaneConverter::convertMeasurementToGlobal(SensorData &Measurement)
  {
    /** check if the measurement has the right type */
    if(Measurement.getType() != SensorType::Pseudorange3)
    {
      PRINT_ERROR("Wrong sensor Type: ", Measurement.getType());
      return;
    }

    /** convert sat pos */
    Vector3 SatPosGlobal = convertToGlobal(Measurement.getValue(SensorElement::SatPos));
    Measurement.setValue(SensorElement::SatPos, SatPosGlobal);

    /** add earth rotation effect again*/
    Vector1 RelCor;
    RelCor(0) = RelativisticCorrection(_TangentPoint.data(), SatPosGlobal);
    Measurement.setMean(Measurement.getMean() + RelCor);
  }

  void TangentPlaneConverter::convertAllPseudorangesToLocal(SensorDataSet &Measurements)
  {
    double Timestamp;

    /** check if right measurements are available */
    if(!Measurements.getTimeFirst(SensorType::Pseudorange3, Timestamp))
    {
      PRINT_ERROR("There is no pseudorange measurement!");
      return;
    }

    /** iterate over all measurements and convert them */
    do
    {
      int N = Measurements.countElement(SensorType::Pseudorange3, Timestamp);
      for (int n = 0; n < N; n++)
      {
        convertMeasurementToLocal(Measurements.getElement(SensorType::Pseudorange3, Timestamp, n));
      }
    }
    while (Measurements.getTimeNext(SensorType::Pseudorange3, Timestamp, Timestamp));
  }

  void TangentPlaneConverter::convertAllStatesToLocal(StateDataSet &States, std::string ID)
  {
    double Timestamp;

    /** check if right States are available */
    if(!States.getTimeFirst(ID, Timestamp))
    {
      PRINT_ERROR("There is no position state!");
      return;
    }

    /** iterate over all States and convert them */
    do
    {
      int N = States.countElement(ID, Timestamp);
      for (int n = 0; n < N; n++)
      {
        convertStateToLocal(States.getElement(ID, Timestamp, n));
      }
    }
    while (States.getTimeNext(ID, Timestamp, Timestamp));
  }

  void TangentPlaneConverter::convertAllStatesToGlobal(StateDataSet &States, std::string ID)
  {
    double Timestamp;

    /** check if right States are available */
    if(!States.getTimeFirst(ID, Timestamp))
    {
      PRINT_ERROR("There is no position state!");
      return;
    }

    /** iterate over all States and convert them */
    do
    {
      int N = States.countElement(ID, Timestamp);
      for (int n = 0; n < N; n++)
      {
        convertStateToGlobal(States.getElement(ID, Timestamp, n));
      }
    }
    while (States.getTimeNext(ID, Timestamp, Timestamp));
  }

  bool TangentPlaneConverter::isInitialized()
  {
    return _isInitialized;
  }

}
