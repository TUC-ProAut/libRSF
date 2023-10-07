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

#include "GNSS.h"

namespace libRSF
{
  TangentPlaneConverter::TangentPlaneConverter()
  {
    isInitialized_ = false;
  }

  TangentPlaneConverter::TangentPlaneConverter(const Vector3 &TangentPoint)
  {
    setTangentPoint(TangentPoint);
  }

  void TangentPlaneConverter::setTangentPoint(Vector3 TangentPoint)
  {
    /** convert to LLA */
    double Lat0, Lon0, H0;
    Earth_.Reverse(TangentPoint(0), TangentPoint(1), TangentPoint(2), Lat0, Lon0, H0);

    /** init converter */
    LocalProjection_.Reset(Lat0, Lon0, H0);

    TangentPoint_ = TangentPoint;
    isInitialized_ = true;
  }

  Vector3 TangentPlaneConverter::convertToLocal_(const Vector3 &GlobalPoint) const
  {
    /** check for initialization*/
    if(!isInitialized_)
    {
      PRINT_WARNING("Converter is not initialized!");
      return GlobalPoint;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    Vector3 LocalPoint;
    Earth_.Reverse(GlobalPoint(0), GlobalPoint(1), GlobalPoint(2), Lat, Lon, H);
    LocalProjection_.Forward(Lat, Lon, H, X, Y, Z);
    LocalPoint << X, Y, Z;

    return LocalPoint;
  }

  Vector3 TangentPlaneConverter::convertToGlobal_(const Vector3 &LocalPoint) const
  {
    /** check for initialization*/
    if(!isInitialized_)
    {
      PRINT_WARNING("Converter is not initialized!");
      return LocalPoint;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    Vector3 GlobalPoint;

    LocalProjection_.Reverse(LocalPoint(0), LocalPoint(1), LocalPoint(2), Lat, Lon, H);
    Earth_.Forward(Lat, Lon, H, X, Y, Z);
    GlobalPoint << X, Y, Z;

    return GlobalPoint;
  }

  void TangentPlaneConverter::convertToLocal_(const Vector3   &GlobalPoint,
                                             const Matrix33  &GlobalCov,
                                                   Vector3   &LocalPoint,
                                                   Matrix33  &LocalCov) const
  {
    /** check for initialization*/
    if(!isInitialized_)
    {
      PRINT_WARNING("Converter is not initialized!");
      return;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    std::vector<double> R1(9);
    std::vector<double> R2(9);

    /** calculate transformation */
    Earth_.Reverse(GlobalPoint(0), GlobalPoint(1), GlobalPoint(2), Lat, Lon, H, R1);
    LocalProjection_.Forward(Lat, Lon, H, X, Y, Z, R2);

    /** convert to eigen */
    Matrix33 R1e(R1.data());
    Matrix33 R2e(R2.data());

    LocalPoint << X, Y, Z;
    LocalCov = R2e*R1e*GlobalCov*(R2e*R1e).transpose();
  }

  void TangentPlaneConverter::convertToGlobal_(const Vector3   &LocalPoint,
                                              const Matrix33  &LocalCov,
                                                    Vector3   &GlobalPoint,
                                                    Matrix33  &GlobalCov) const
  {
    /** check for initialization*/
    if(!isInitialized_)
    {
      PRINT_WARNING("Converter is not initialized!");
      return;
    }

    double Lat, Lon, H;
    double X,Y,Z;
    std::vector<double> R1(9);
    std::vector<double> R2(9);

    /** calculate transformation */
    LocalProjection_.Reverse(LocalPoint(0), LocalPoint(1), LocalPoint(2), Lat, Lon, H, R1);
    Earth_.Forward(Lat, Lon, H, X, Y, Z, R2);


    /** convert to eigen */
    Matrix33 R1e(R1.data());
    Matrix33 R2e(R2.data());

    GlobalPoint << X, Y, Z;
    GlobalCov = R2e*R1e*LocalCov*(R2e*R1e).transpose();
  }


  void TangentPlaneConverter::convertStateToLocal(Data &State)
  {
    /** check if the state has the right type */
    if(State.getType() != DataType::Point3)
    {
      PRINT_ERROR("Wrong state Type: ", State.getType());
      return;
    }

    /** extract values */
    const Vector3 MeanIn = State.getMean();
    const Matrix33 CovIn = State.getCovarianceMatrix();

    /** convert */
    Vector3 MeanOut;
    Matrix33 CovOut;
    this->convertToLocal_(MeanIn, CovIn, MeanOut, CovOut);

    /** write back */
    Vector9 CovOutVect(CovOut.data());
    State.setMean(MeanOut);
    State.setCovariance(CovOutVect);
  }

  void TangentPlaneConverter::convertStateToGlobal(Data &State)
  {
    /** check if the state has the right type */
    if(State.getType() != DataType::Point3)
    {
      PRINT_ERROR("Wrong state Type: ", State.getType());
      return;
    }

    /** extract values */
    const Vector3 MeanIn = State.getMean();
    const Matrix33 CovIn = State.getCovarianceMatrix();

    /** convert */
    Vector3 MeanOut;
    Matrix33 CovOut;
    this->convertToGlobal_(MeanIn, CovIn, MeanOut, CovOut);

    /** write back */
    Vector9 CovOutVect(CovOut.data());
    State.setMean(MeanOut);
    State.setCovariance(CovOutVect);
  }

  void TangentPlaneConverter::convertMeasurementToLocal(Data &Measurement)
  {
    /** check if the measurement has the right type */
    if(Measurement.getType() != DataType::Pseudorange3)
    {
      PRINT_ERROR("Wrong sensor Type: ", Measurement.getType());
      return;
    }

    /** remove earth rotation effect */
    Vector3 SatPosGlobal = Measurement.getValue(DataElement::SatPos);
    Vector1 RelCor;
    RelCor(0) = RelativisticCorrection(TangentPoint_.data(), SatPosGlobal);
    Measurement.setMean(Measurement.getMean() - RelCor);

    /** convert sat pos */
    Measurement.setValue(DataElement::SatPos, convertToLocal_(SatPosGlobal));
  }

  void TangentPlaneConverter::convertMeasurementToGlobal(Data &Measurement)
  {
    /** check if the measurement has the right type */
    if(Measurement.getType() != DataType::Pseudorange3)
    {
      PRINT_ERROR("Wrong sensor Type: ", Measurement.getType());
      return;
    }

    /** convert sat pos */
    Vector3 SatPosGlobal = convertToGlobal_(Measurement.getValue(DataElement::SatPos));
    Measurement.setValue(DataElement::SatPos, SatPosGlobal);

    /** add earth rotation effect again*/
    Vector1 RelCor;
    RelCor(0) = RelativisticCorrection(TangentPoint_.data(), SatPosGlobal);
    Measurement.setMean(Measurement.getMean() + RelCor);
  }

  void TangentPlaneConverter::convertAllPseudorangesToLocal(SensorDataSet &Measurements)
  {
    double Timestamp;

    /** check if right measurements are available */
    if(!Measurements.getTimeFirst(DataType::Pseudorange3, Timestamp))
    {
      PRINT_ERROR("There is no pseudorange measurement!");
      return;
    }

    /** iterate over all measurements and convert them */
    do
    {
      int N = Measurements.countElement(DataType::Pseudorange3, Timestamp);
      for (int n = 0; n < N; n++)
      {
        convertMeasurementToLocal(Measurements.getElement(DataType::Pseudorange3, Timestamp, n));
      }
    }
    while (Measurements.getTimeNext(DataType::Pseudorange3, Timestamp, Timestamp));
  }

  void TangentPlaneConverter::convertAllStatesToLocal(StateDataSet &States, const std::string& ID)
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

  void TangentPlaneConverter::convertAllStatesToGlobal(StateDataSet &States, const std::string& ID)
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

  bool TangentPlaneConverter::isInitialized() const
  {
    return isInitialized_;
  }

}
